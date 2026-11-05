# MKS CAN 신뢰성 patch4 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** MKS ODrive v3.6 펌웨어 patch3를 patch4로 올려, 정보성 CAN 오류는 모터를 세우지 않고 CAN 제어 수신이 끊긴 축만 보드가 스스로 세우게 한다.

**Architecture:** 저장소에는 벤더 소스가 아니라 `firmware/mks_odrive/reliability.patch`(벤더 ZIP 대비 unified diff)만 둔다. 개발은 저장소 밖 작업 트리에서 벤더 소스를 직접 고치고, 새 도구 `regen_patch.py`로 패치를 다시 만든다. 검증은 실제 벤더 함수를 추출해 결정적 HAL/RTOS 경계와 함께 컴파일하는 기존 소스 하네스(`tests/run_source_tests.py`)를 확장해서 한다. 호스트(Jetson) 코드는 바꾸지 않는다.

**Tech Stack:** C++17(g++, 호스트 하네스), STM32 HAL F4·FreeRTOS(펌웨어), Python 3 표준 라이브러리(도구·시험), Docker 고정 이미지(Cortex-M4 빌드), python-can(Jetson 실측 도구).

**Spec:** `docs/specs/2026-10-15-mks-can-patch4.md` (커밋 `8a71eb5`). 이 계획의 D0~D5는 설계 문서의 같은 이름 절을 구현한다. 실행자는 두 문서를 함께 읽는다.

## Global Constraints

- patch 식별자 `can.reliability_patch == 4`. 펌웨어 버전 문자열은 `0.5.1-dev`, `fw_version_unreleased=1` 유지.
- CAN 와이어 포맷 불변: heartbeat byte4 axis state, byte5 `0xA3`, byte6 flags bit0 latch·bit1 in-progress·bit2 eligible, byte7 generation; cmd `0x07` guarded RESUME `<IBBBB>` = `(8, 0xA3, generation, 0, 0)`; cmd `0x18` clear는 zero8/DLC0만.
- `Config_t`/NVM 레이아웃 불변(`tests/test_prepare.py`의 `config_body` 비교가 계속 통과해야 함).
- D0: `T_cmd = 300 ms`, `T_grace = 500 ms`, 비교는 `>`(tick 차분 301/501부터 만료), 시계는 `osKernelSysTick()`을 `osKernelSysTickFrequency`로 ms 환산, unsigned 차분. CLOSED_LOOP에서만 검사. `enable_watchdog`과 무관. 만료 = `ERROR_WATCHDOG_TIMER_EXPIRED`, 자동 재개 자격 없음.
- D1: 정보성 mask `0x000007FF`(EWG `0x1`·EPV `0x2`·BOF `0x4`·STF `0x8`·FOR `0x10`·ACK `0x20`·BR `0x40`·BD `0x80`·CRC `0x100`·RX_FOV0 `0x200`·RX_FOV1 `0x400`). ISR은 재초기화를 요청하지 않는다.
- D2: STD id word `node << 26`, mask `0xFC000004`, node ≤ `0x3F`. EXT id word `(node << 8) | 4`, mask `0xFFFFFF04`, node ≤ `0xFFFFFF`. bank0=axis0, bank1=axis1, FIFO0, `SlaveStartFilterBank=14`. 송신: ID 생성 전 node 범위 검사 + `write()`에서 STD ID ≤ `0x7FF`, EXT ID ≤ `0x1FFFFFFF`.
- D4: 축별 래치 에피소드 시도 수 포화 카운터, `K = 16`, 17번째 시도부터 그 에피소드 자격 비복구.
- D5: 래치(`latch_can_fault`)는 CAN 서버 스레드에서만 건다.
- 호스트 코드(`motor_control/`, `ros2/`) 변경 없음. node11/12 보드는 범위 밖.
- 실물 플래시·NVM 접근·로봇 구동은 **사용자 별도 승인 후에만**. 커밋·push는 매번 사용자 확인 후, 파일 단위 `git add`만.
- 교차검증: 각 커밋 전에 Codex 리뷰를 받고 새 P1/P2가 없을 때까지 반복한다(아래 각 Task의 리뷰 단계).

## Review Focus

설계 문서가 직접 시험을 요구하지 않았지만 실제 운용에서 가장 먼저 부딪힐 입력 다섯 가지다. 각 줄의 시험은 해당 Task에 들어 있다.

1. 운전 중이 아닐 때 USB로 node ID를 바꾸면 수신 필터가 새 node를 따라가야 한다 → Task 3 `filter_usb_node_change`.
2. 버스 잡음으로 LEC 인터럽트가 수천 번 연속하면 정지·래치 없이 계수만 오르고 알림이 다시 켜져야 한다 → Task 2 `info_storm`.
3. 호스트가 정확히 300 ms에서 멈췄다 재개하면 경계값에서 만료하지 않고 301 ms에서 만료해야 한다 → Task 6 `d0_cmd_timeout`.
4. 래치 에피소드 도중 MCU가 재부팅되면 새 RAM 필드는 모두 기본값이어야 한다 → Task 6 `d0_reboot_defaults`.
5. 한 보드의 두 축이 서로 다른 소유(CAN/USB)로 돌면 만료가 축별로 독립이어야 한다 → Task 6 `d0_per_axis`.

---

## 공통 준비 (모든 Task가 사용하는 셸 변수)

모든 명령은 저장소 루트에서 실행한다. 셸은 bash 기준이다(zsh는 `for` 단어 분할이 다르다).

```bash
export REPO=$(git rev-parse --show-toplevel)
export PY=${PY:-python3}                         # 하네스·도구는 표준 라이브러리만 쓴다
export WORK=$HOME/.cache/zetin-mks-patch4        # 저장소 밖 작업 트리
export ZIP=$WORK/ODrive-fw-v0.5.1.zip
```

- `$WORK/original` = 패치 없는 벤더 소스(RED 기준)
- `$WORK/patch3` = 현재 커밋된 patch3 적용 소스(음성 대조, 수정 금지)
- `$WORK/work` = patch4를 개발하는 소스(Task 2~7에서 수정)

하네스 실행 명령(이하 "하네스 실행"):

```bash
$PY firmware/mks_odrive/tests/run_source_tests.py $WORK/work      # 개발본
$PY firmware/mks_odrive/tests/run_source_tests.py $WORK/patch3    # 음성 대조
$PY firmware/mks_odrive/tests/run_source_tests.py $WORK/original  # 원본
```

---

### Task 0: 작업 환경과 패치 재생성 도구

**Files:**
- Create: `firmware/mks_odrive/regen_patch.py`
- Create: `firmware/mks_odrive/tests/test_regen_patch.py`

**Interfaces:**
- Produces: `regen_patch.PATCH_FILES: list[str]`(패치에 들어가는 9개 경로, 현재 패치 순서), `regen_patch.regenerate(original: Path, work: Path, output: Path) -> None`. CLI `python regen_patch.py --original DIR --work DIR [--output PATH]`.

- [ ] **Step 1: 브랜치와 작업 트리 준비**

```bash
git switch -c feat/mks-can-patch4
mkdir -p $WORK
curl -sSL -o $ZIP 'https://raw.githubusercontent.com/makerbase-motor/MKS-ODrive/e15782976ae93d42b1f0648ceec96503141a343b/Firmware/ODrive_V3.6/ODrive-fw-v0.5.1.zip'
sha256sum $ZIP
```
Expected: `1c9ff347f997bbbf8cb693cdb44d93fe6f6ef992aefde60b6a65b0e1fa25a777`. 다르면 중단한다.

```bash
$PY firmware/mks_odrive/prepare_source.py --zip $ZIP --dest $WORK/original --unpatched
$PY firmware/mks_odrive/prepare_source.py --zip $ZIP --dest $WORK/patch3
cp -a $WORK/patch3 $WORK/work && rm $WORK/work/mks-source-manifest.json
$PY firmware/mks_odrive/tests/run_source_tests.py $WORK/patch3 | tail -1
$PY firmware/mks_odrive/tests/run_source_tests.py $WORK/original | tail -1
```
Expected: `86 passed, 0 failed; PATCHED=True` / `4 passed, 82 failed; PATCHED=False`.

- [ ] **Step 2: 실패하는 시험 작성** — `firmware/mks_odrive/tests/test_regen_patch.py`

```python
#!/usr/bin/env python3
"""regen_patch must reproduce a source tree exactly through prepare_source."""
import os
from pathlib import Path
import sys
import tempfile
import unittest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import prepare_source  # noqa: E402
import regen_patch  # noqa: E402

ARCHIVE = Path(os.environ.get('MKS_VENDOR_ZIP', ''))


def tree(root):
    return {str(p.relative_to(root)): p.read_bytes() for p in sorted(root.rglob('*'))
            if p.is_file() and p.name != 'mks-source-manifest.json'}


@unittest.skipUnless(ARCHIVE.is_file(), 'set MKS_VENDOR_ZIP to the pinned vendor ZIP')
class RegenPatchTest(unittest.TestCase):
    def test_regenerated_patch_reproduces_work_tree(self):
        with tempfile.TemporaryDirectory() as temp:
            temp = Path(temp)
            prepare_source.prepare(ARCHIVE, temp / 'original', None)
            prepare_source.prepare(ARCHIVE, temp / 'work', HERE.parent / 'reliability.patch')
            out = temp / 'regen.patch'
            regen_patch.regenerate(temp / 'original', temp / 'work', out)
            prepare_source.prepare(ARCHIVE, temp / 'check', out)
            self.assertEqual(tree(temp / 'work'), tree(temp / 'check'))

    def test_file_list_matches_committed_patch(self):
        text = (HERE.parent / 'reliability.patch').read_text()
        named = [line[6:] for line in text.splitlines() if line.startswith('+++ b/')]
        self.assertEqual(named, regen_patch.PATCH_FILES)


if __name__ == '__main__':
    unittest.main(verbosity=2)
```

- [ ] **Step 3: 실패 확인**

Run: `MKS_VENDOR_ZIP=$ZIP $PY firmware/mks_odrive/tests/test_regen_patch.py`
Expected: `ModuleNotFoundError: No module named 'regen_patch'`

- [ ] **Step 4: 구현** — `firmware/mks_odrive/regen_patch.py`

```python
#!/usr/bin/env python3
"""Regenerate reliability.patch from an unpatched and an edited MKS source tree."""
import argparse
from pathlib import Path
import subprocess

PATCH_FILES = [
    'Firmware/communication/interface_can.cpp',
    'Firmware/communication/interface_can.hpp',
    'Firmware/communication/can_simple.cpp',
    'Firmware/MotorControl/axis.hpp',
    'Firmware/MotorControl/axis.cpp',
    'Firmware/MotorControl/low_level.cpp',
    'Firmware/odrive-interface.yaml',
    'Firmware/Tupfile.lua',
    'tools/odrive/mks_reliability_version.py',
]


def regenerate(original, work, output):
    chunks = []
    for rel in PATCH_FILES:
        old, new = original / rel, work / rel
        if not new.is_file():
            raise FileNotFoundError(new)
        if old.is_file():
            cmd = ['diff', '-u', '--label', 'a/' + rel, '--label', 'b/' + rel, str(old), str(new)]
        else:
            cmd = ['diff', '-u', '--label', '/dev/null', '--label', 'b/' + rel, '/dev/null', str(new)]
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode not in (0, 1):  # diff: 0 same, 1 different, 2 trouble
            raise RuntimeError(result.stderr)
        if result.returncode == 0:
            raise ValueError(f'{rel} has no patch change; remove it from PATCH_FILES deliberately')
        chunks.append(result.stdout)
    output.write_text(''.join(chunks))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--original', type=Path, required=True)
    ap.add_argument('--work', type=Path, required=True)
    ap.add_argument('--output', type=Path, default=Path(__file__).with_name('reliability.patch'))
    args = ap.parse_args()
    regenerate(args.original, args.work, args.output)
    print(args.output.resolve())


if __name__ == '__main__':
    main()
```

- [ ] **Step 5: 통과 확인**

Run: `MKS_VENDOR_ZIP=$ZIP $PY firmware/mks_odrive/tests/test_regen_patch.py`
Expected: 2 tests OK.

Run: `$PY firmware/mks_odrive/regen_patch.py --original $WORK/original --work $WORK/work --output /tmp/regen-check.patch && diff -q /tmp/regen-check.patch firmware/mks_odrive/reliability.patch && echo IDENTICAL`
Expected: `IDENTICAL`이 이상적이다. 다르면 형식 차이일 뿐이며(Step 5 첫 시험이 의미 동등성을 보장), 커밋된 patch3 파일은 이 Task에서 바꾸지 않는다.

- [ ] **Step 6: 커밋 보류** — Task 1과 함께 커밋한다(Task 1 Step 9).

---

### Task 1: 하네스를 실제 CAN IRQ·FIFO·필터와 실제 축 상태머신으로 확장

동작 변경 없이 하네스만 넓힌다. patch3·원본의 기존 86개 결과는 그대로이고, 자체 시험 2개가 더해진다.
이 Task의 변경 전체는 계획 작성 중 저장소 밖 사본에서 실행해 검증한 패치다(아래 "자체 검토 기록" ①).

**Files:**
- Modify: `firmware/mks_odrive/tests/hal_boundary.hpp`
- Modify: `firmware/mks_odrive/tests/run_source_tests.py`
- Modify: `firmware/mks_odrive/tests/scenarios.cpp`

**Interfaces:**
- Produces(하네스, 이후 모든 Task가 사용):
  - 실제 CMSIS/HAL CAN 상수(`/* REAL_CAN_DEFINES */`에 러너가 주입), 실제 `HAL_CAN_IRQHandler` 링크.
  - `uint32_t virtual_tick`(기본 100), `osKernelSysTickFrequency = 1000`, `uint32_t calibration_ms`(캘리 스텁이 소모하는 tick 수).
  - `bool deliver(uint32_t id, bool ext=false, bool rtr=false)` — 기록된 필터 bank로 수락 판정 후 FIFO0(깊이 3, 가득 차면 마지막 메시지 덮어쓰기 + `CAN_RF0R_FOVR0`)에 넣는다. 반환값은 필터 수락 여부(FIFO 용량과 무관).
  - `void fire_can_irq()`(critical 밖에서만), `void inject_esr(uint32_t esr)`, `CAN_FilterTypeDef filters[28]`, `uint32_t filter_config_calls`, `bool fail_filter`, `int sem_releases`, `std::function<void()> on_wait`(CAN 서버 대기마다), `std::function<void()> meas_hook`(축 루프 `wait_for_current_meas`마다, tick 1 증가 뒤 호출).
  - 실제 `run_control_loop`·`run_closed_loop_control_loop`·`run_idle_loop`·`run_state_machine_loop` 추출. 하네스 `Axis`의 `current_state_`는 실제처럼 `task_chain_.front()` 참조다.
  - 러너 매크로 `PATCH4`(1 = 소스에 `take_informational_errors`가 있음).

- [ ] **Step 1: 하네스 패치 적용** — 아래 내용을 `/tmp/task1-harness.patch`로 저장하고 저장소 루트에서 적용한다.

```bash
git apply --check /tmp/task1-harness.patch && git apply /tmp/task1-harness.patch
```

```diff
diff --color -ruN a/firmware/mks_odrive/tests/hal_boundary.hpp b/firmware/mks_odrive/tests/hal_boundary.hpp
--- a/firmware/mks_odrive/tests/hal_boundary.hpp
+++ b/firmware/mks_odrive/tests/hal_boundary.hpp
@@ -9,21 +9,24 @@
 #include <limits>
 #include <functional>
 #include <vector>
+#include <array>
 using HAL_StatusTypeDef=int;
 constexpr int HAL_OK=0,HAL_ERROR=1,HAL_CAN_STATE_RESET=0,HAL_CAN_STATE_READY=1,HAL_CAN_STATE_LISTENING=2;
-constexpr uint32_t HAL_CAN_ERROR_NONE=0,HAL_CAN_ERROR_TIMEOUT=0x20000,HAL_CAN_ERROR_PARAM=0x00200000,HAL_CAN_ERROR_NOT_INITIALIZED=0x00040000;
-constexpr uint32_t CAN_ID_STD=0,CAN_ID_EXT=4,CAN_RTR_DATA=0,RESET=0;
-constexpr uint32_t CAN_TSR_TME0=1u<<26,CAN_TSR_TME1=1u<<27,CAN_TSR_TME2=1u<<28,CAN_TSR_CODE=3u<<24,CAN_TSR_CODE_Pos=24;
-constexpr uint32_t CAN_TI0R_STID_Pos=21,CAN_TI0R_EXID_Pos=3,CAN_TI0R_TXRQ=1,CAN_TDT0R_TGT=8;
-constexpr int CAN_TDH0R_DATA7_Pos=24,CAN_TDH0R_DATA6_Pos=16,CAN_TDH0R_DATA5_Pos=8,CAN_TDH0R_DATA4_Pos=0;
-constexpr int CAN_TDL0R_DATA3_Pos=24,CAN_TDL0R_DATA2_Pos=16,CAN_TDL0R_DATA1_Pos=8,CAN_TDL0R_DATA0_Pos=0;
-constexpr uint32_t CAN_RX_FIFO0=0,CAN_RX_FIFO1=1,CAN_IT_RX_FIFO0_MSG_PENDING=1,CAN_IT_ERROR=2,CAN_IT_BUSOFF=4,CAN_IT_LAST_ERROR_CODE=8,CAN_IT_ERROR_WARNING=16,CAN_IT_ERROR_PASSIVE=32,CAN_IT_RX_FIFO0_OVERRUN=64,CAN_IT_RX_FIFO1_OVERRUN=128,CAN_TX_MAILBOX0=1,CAN_TX_MAILBOX1=2,CAN_TX_MAILBOX2=4;
-constexpr int CAN_FILTERMODE_IDMASK=0,CAN_FILTERSCALE_32BIT=0;
+constexpr uint32_t RESET=0;
+/* REAL_CAN_DEFINES */
 enum FunctionalState {DISABLE=0,ENABLE=1};
 struct Mailbox {uint32_t TIR=0,TDTR=0,TDHR=0,TDLR=0;};
-struct Registers {uint32_t TSR=CAN_TSR_TME0;Mailbox sTxMailBox[3];} regs;
+struct Registers {uint32_t TSR=CAN_TSR_TME0;uint32_t IER=0,ESR=0,MSR=0,RF0R=0,RF1R=0;Mailbox sTxMailBox[3];} regs;
 Registers* const CAN1=&regs;
 struct CAN_HandleTypeDef {int State=HAL_CAN_STATE_LISTENING;uint32_t ErrorCode=0;Registers* Instance=&regs;struct {uint32_t Prescaler=4;} Init;};
+#define READ_REG(r) (r)
+#define CLEAR_BIT(r,b) ((r)&=~(b))
+void can_clear_flag(CAN_HandleTypeDef* hc,uint32_t flag){const uint32_t bit=1u<<(flag&CAN_FLAG_MASK);
+ switch(flag>>8U){case 5:hc->Instance->TSR&=~bit;break;case 2:hc->Instance->RF0R&=~bit;break;
+  case 4:hc->Instance->RF1R&=~bit;break;case 1:hc->Instance->MSR&=~bit;break;default:break;}}
+#define __HAL_CAN_CLEAR_FLAG(h,f) can_clear_flag((h),(f))
+struct RxFrame{uint32_t id;bool ext,rtr;};
+std::vector<RxFrame> rx_fifo0;
 struct CAN_TxHeaderTypeDef {uint32_t StdId,ExtId,IDE,RTR,DLC;FunctionalState TransmitGlobalTime;};
 struct CAN_RxHeaderTypeDef {uint32_t StdId=0,ExtId=0,IDE=0,RTR=0,DLC=8;};
 struct CAN_FilterTypeDef {uint32_t FilterActivation=0,FilterBank=0,FilterFIFOAssignment=0,FilterIdHigh=0,FilterIdLow=0,FilterMaskIdHigh=0,FilterMaskIdLow=0,FilterMode=0,FilterScale=0,SlaveStartFilterBank=0;};
@@ -33,6 +36,12 @@
 uint8_t observed_tx[8] = {}; uint32_t observed_tx_id=0; uint8_t observed_tx_len=0;
 template<class Message> void observe_tx_attempt(const Message& msg){std::memcpy(observed_tx,msg.buf,8);observed_tx_id=msg.id;observed_tx_len=msg.len;}
 uint32_t fail_inits=0, fifo[2]={0,0}, can_irqs_disabled=0, rcc_reset_count=0, pending_irq_cleared=0;
+uint32_t virtual_tick=100, filter_config_calls=0;int sem_releases=0;bool fail_filter=false;
+constexpr uint32_t osKernelSysTickFrequency=1000;
+std::function<void()> on_wait, meas_hook;
+uint32_t calibration_ms=0;
+CAN_HandleTypeDef* irq_handle=nullptr;
+void HAL_CAN_IRQHandler(CAN_HandleTypeDef*);
 bool can_clock_enabled=true;
 int wait_budget=1;
 bool reinit_active=false, fail_stop=false;
@@ -51,28 +60,42 @@
 #define SET_BIT(r,v) set_bit(r,v)
 #define WRITE_REG(r,v) ((r)=(v))
 uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef*,uint32_t n){return fifo[n];}
-HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef*,uint32_t n,CAN_RxHeaderTypeDef* header,uint8_t* buf){++rx_reads;if(!fifo[n])return HAL_ERROR;--fifo[n];*header=CAN_RxHeaderTypeDef{};std::memset(buf,0,8);return HAL_OK;}
+HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef*,uint32_t n,CAN_RxHeaderTypeDef* header,uint8_t* buf){
+ ++rx_reads;if(!fifo[n])return HAL_ERROR;--fifo[n];*header=CAN_RxHeaderTypeDef{};std::memset(buf,0,8);
+ if(n==0 && !rx_fifo0.empty()){RxFrame f=rx_fifo0.front();rx_fifo0.erase(rx_fifo0.begin());
+  header->IDE=f.ext?CAN_ID_EXT:CAN_ID_STD;header->StdId=f.id;header->ExtId=f.id;header->RTR=f.rtr?CAN_RTR_REMOTE:CAN_RTR_DATA;}
+ regs.RF0R=(regs.RF0R&~3u)|(fifo[0]&3u);return HAL_OK;}
 HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef* h){require(!irq_mask,"HAL Stop masks motor IRQs");++stop_calls;reinit_active=true;if(during_reinit)during_reinit();if(fail_stop){h->ErrorCode|=HAL_CAN_ERROR_TIMEOUT;return HAL_ERROR;}h->State=HAL_CAN_STATE_READY;return HAL_OK;}
 HAL_StatusTypeDef HAL_CAN_DeInit(CAN_HandleTypeDef* h){HAL_CAN_Stop(h);can_clock_enabled=false;h->State=HAL_CAN_STATE_RESET;h->ErrorCode=0;return HAL_OK;}
 constexpr int CAN1_TX_IRQn=0,CAN1_RX0_IRQn=1,CAN1_RX1_IRQn=2,CAN1_SCE_IRQn=3;
 void HAL_NVIC_DisableIRQ(int irq){can_irqs_disabled|=1u<<irq;}
 void HAL_NVIC_ClearPendingIRQ(int irq){pending_irq_cleared|=1u<<irq;}
-void rcc_force_reset(){require(!irq_mask,"RCC reset masks motor IRQs");require(can_clock_enabled,"reset with CAN clock disabled");require(can_irqs_disabled==15,"CAN IRQ may run during RCC reset");++rcc_reset_count;fifo[0]=fifo[1]=0;regs.TSR=CAN_TSR_TME0|CAN_TSR_TME1|CAN_TSR_TME2;}
+void rcc_force_reset(){require(!irq_mask,"RCC reset masks motor IRQs");require(can_clock_enabled,"reset with CAN clock disabled");require(can_irqs_disabled==15,"CAN IRQ may run during RCC reset");++rcc_reset_count;fifo[0]=fifo[1]=0;rx_fifo0.clear();regs.RF0R=regs.RF1R=regs.ESR=regs.MSR=regs.IER=0;regs.TSR=CAN_TSR_TME0|CAN_TSR_TME1|CAN_TSR_TME2;}
 #define __HAL_RCC_CAN1_FORCE_RESET() rcc_force_reset()
 #define __HAL_RCC_CAN1_RELEASE_RESET() ((void)0)
 #define __DSB() ((void)0)
 HAL_StatusTypeDef HAL_CAN_Init(CAN_HandleTypeDef* h){require(!irq_mask,"HAL Init masks motor IRQs");++init_calls;can_clock_enabled=true;can_irqs_disabled=0;if(fail_inits){--fail_inits;h->ErrorCode|=HAL_CAN_ERROR_TIMEOUT;return HAL_ERROR;}h->State=HAL_CAN_STATE_READY;h->ErrorCode=0;return HAL_OK;}
 HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef* h){require(!irq_mask,"HAL Start masks motor IRQs");h->State=HAL_CAN_STATE_LISTENING;reinit_active=false;return HAL_OK;}
 HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef*,uint32_t){regs.TSR=CAN_TSR_TME0|CAN_TSR_TME1|CAN_TSR_TME2;return HAL_OK;}
-HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef*,uint32_t){return HAL_OK;}
-HAL_StatusTypeDef HAL_CAN_DeactivateNotification(CAN_HandleTypeDef*,uint32_t){return HAL_OK;}
-HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef*,CAN_FilterTypeDef*){return HAL_OK;}
+HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef* hc,uint32_t it){hc->Instance->IER|=it;return HAL_OK;}
+HAL_StatusTypeDef HAL_CAN_DeactivateNotification(CAN_HandleTypeDef* hc,uint32_t it){hc->Instance->IER&=~it;return HAL_OK;}
+CAN_FilterTypeDef filters[28];
+HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef*,CAN_FilterTypeDef* f){++filter_config_calls;if(fail_filter)return HAL_ERROR;
+ require(f->FilterBank<28,"filter bank out of range");filters[f->FilterBank]=*f;return HAL_OK;}
+bool filter_accepts(const RxFrame& f){uint32_t word=f.ext?((f.id<<3)|4u):(f.id<<21);if(f.rtr)word|=2u;
+ for(const auto& b:filters){if(!b.FilterActivation)continue;const uint32_t id=(b.FilterIdHigh<<16)|b.FilterIdLow,mask=(b.FilterMaskIdHigh<<16)|b.FilterMaskIdLow;
+  if(((word^id)&mask)==0)return true;}return false;}
+bool deliver(uint32_t id,bool ext=false,bool rtr=false){const RxFrame f{id,ext,rtr};if(!filter_accepts(f))return false;
+ if(fifo[0]<3){rx_fifo0.push_back(f);++fifo[0];}else{if(!rx_fifo0.empty())rx_fifo0.back()=f;regs.RF0R|=CAN_RF0R_FOVR0;}
+ regs.RF0R=(regs.RF0R&~3u)|(fifo[0]&3u);return true;}
+void fire_can_irq(){require(!irq_mask,"CAN IRQ fired inside a critical section");HAL_CAN_IRQHandler(irq_handle);}
+void inject_esr(uint32_t esr){regs.ESR=esr;regs.MSR|=CAN_MSR_ERRI;}
 HAL_StatusTypeDef HAL_CAN_ResetError(CAN_HandleTypeDef* h){h->ErrorCode=0;++reset_calls;return HAL_OK;}
 using osThreadId=void*;using StackType_t=uint32_t;
 constexpr int sem_can=1,osPriorityNormal=0;
-uint32_t osKernelSysTick(){return 100;}
-int osSemaphoreRelease(int){return 0;}
-void boundary_wait(){require(!irq_mask,"RTOS wait with motor IRQs masked");error_reads=0;if(++waits>=uint32_t(wait_budget))throw StopLoop{};}
+uint32_t osKernelSysTick(){return virtual_tick;}
+int osSemaphoreRelease(int){++sem_releases;return 0;}
+void boundary_wait(){require(!irq_mask,"RTOS wait with motor IRQs masked");error_reads=0;if(on_wait){auto f=on_wait;f();}if(++waits>=uint32_t(wait_budget))throw StopLoop{};}
 int osSemaphoreWait(int,int timeout){require(timeout>0,"unbounded wait");boundary_wait();return 0;}
 void osDelay(int delay){require(delay>0 && delay<=100,"missing or excessive backoff");++delay_count;boundary_wait();}
 #define osThreadDef(name,func,priority,n,stack) int name=0
@@ -82,20 +105,28 @@
 namespace ODriveIntf {struct CanIntf {enum Protocol{PROTOCOL_SIMPLE=0};enum Error{ERROR_NONE=0,ERROR_DUPLICATE_CAN_IDS=1};};}
 inline ODriveIntf::CanIntf::Error& operator|=(ODriveIntf::CanIntf::Error& a,ODriveIntf::CanIntf::Error b){a=static_cast<ODriveIntf::CanIntf::Error>(int(a)|int(b));return a;}
 class Axis;
-struct Motor {Axis* axis_=nullptr;struct{Motor* timer=nullptr;}hw_config_;enum {ERROR_NONE=0,ARMED_STATE_DISARMED=0,ARMED_STATE_WAITING_FOR_TIMINGS=1};uint32_t error_=0;int armed_state_=1;bool disarmed=false;struct{float Iq_setpoint=0,Iq_measured=0;} current_control_;};
+struct Motor {Axis* axis_=nullptr;struct{Motor* timer=nullptr;}hw_config_;enum {ERROR_NONE=0,ARMED_STATE_DISARMED=0,ARMED_STATE_WAITING_FOR_TIMINGS=1};uint32_t error_=0;int armed_state_=1;bool disarmed=false;struct{float Iq_setpoint=0,Iq_measured=0;} current_control_;bool is_calibrated_=true;struct{int direction=1;int pole_pairs=10;}config_;bool arm();bool run_calibration();bool update(float,float,float){return true;}};
 bool safety_critical_disarm_motor_pwm(Motor& m);
 void safety_critical_arm_motor_pwm(Motor& m);
+void advance_calibration();
+inline bool Motor::arm(){safety_critical_arm_motor_pwm(*this);return true;}
+inline bool Motor::run_calibration(){advance_calibration();return true;}
 bool brake_resistor_armed=true;
 #define __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(timer) ((timer)->disarmed=true)
-struct Controller {enum ControlMode{CONTROL_MODE_POSITION_CONTROL=3};enum InputMode{INPUT_MODE_PASSTHROUGH=1};enum{ERROR_NONE=0};uint32_t error_=0;float input_pos_=0,input_vel_=0,input_torque_=0;struct{ControlMode control_mode=CONTROL_MODE_POSITION_CONTROL;InputMode input_mode=INPUT_MODE_PASSTHROUGH;float vel_limit=0,inertia=0;}config_;void input_pos_updated(){}void start_anticogging_calibration(){};};
-struct Encoder {enum{ERROR_NONE=0};uint32_t error_=0;float spi_error_rate_=0,pos_estimate_=3,vel_estimate_=0;int shadow_count_=0,count_in_cpr_=0;};
+struct Controller {enum ControlMode{CONTROL_MODE_POSITION_CONTROL=3};enum InputMode{INPUT_MODE_PASSTHROUGH=1};enum{ERROR_NONE=0};uint32_t error_=0;float input_pos_=0,input_vel_=0,input_torque_=0;struct{ControlMode control_mode=CONTROL_MODE_POSITION_CONTROL;InputMode input_mode=INPUT_MODE_PASSTHROUGH;float vel_limit=0,inertia=0;int load_encoder_axis=0;bool circular_setpoints=false;}config_;float pos_setpoint_=0,vel_setpoint_=0,vel_integrator_torque_=0;float* pos_estimate_linear_src_=nullptr;float* pos_estimate_circular_src_=nullptr;bool select_encoder(int){return true;}bool update(float* t){*t=0;return true;}void reset(){}void input_pos_updated(){}void start_anticogging_calibration(){};};
+struct Encoder {enum{ERROR_NONE=0};uint32_t error_=0;float spi_error_rate_=0,pos_estimate_=3,vel_estimate_=0;int shadow_count_=0,count_in_cpr_=0;bool is_ready_=true;float phase_=0;struct{bool use_index=false,idx_search_unidirectional=false;}config_;bool run_index_search(){return true;}bool run_direction_find(){return true;}bool run_offset_calibration(){advance_calibration();return true;}};
 struct SensorlessEstimator {enum{ERROR_NONE=0};uint32_t error_=0;float pll_pos_=0,vel_estimate_=0;};
 struct Axis {
- enum AxisState{AXIS_STATE_UNDEFINED=0,AXIS_STATE_IDLE=1,AXIS_STATE_CLOSED_LOOP_CONTROL=8};
- enum{ERROR_NONE=0,ERROR_ESTOP_REQUESTED=0x4000,ERROR_WATCHDOG_TIMER_EXPIRED=0x800};
- struct{uint32_t can_node_id=13,can_heartbeat_rate_ms=20;bool can_node_id_extended=false,enable_watchdog=true;}config_;
+ enum AxisState{AXIS_STATE_UNDEFINED=0,AXIS_STATE_IDLE=1,AXIS_STATE_STARTUP_SEQUENCE=2,AXIS_STATE_FULL_CALIBRATION_SEQUENCE=3,AXIS_STATE_MOTOR_CALIBRATION=4,AXIS_STATE_SENSORLESS_CONTROL=5,AXIS_STATE_ENCODER_INDEX_SEARCH=6,AXIS_STATE_ENCODER_OFFSET_CALIBRATION=7,AXIS_STATE_CLOSED_LOOP_CONTROL=8,AXIS_STATE_LOCKIN_SPIN=9,AXIS_STATE_ENCODER_DIR_FIND=10,AXIS_STATE_HOMING=11};
+ struct LockinConfig_t{float vel=40.0f;};
+ enum{ERROR_NONE=0,ERROR_INVALID_STATE=0x1,ERROR_CURRENT_MEASUREMENT_TIMEOUT=0x8,ERROR_MOTOR_DISARMED=0x20,ERROR_CONTROLLER_FAILED=0x200,ERROR_WATCHDOG_TIMER_EXPIRED=0x800,ERROR_ESTOP_REQUESTED=0x4000};
+ struct{uint32_t can_node_id=13,can_heartbeat_rate_ms=20;bool can_node_id_extended=false,enable_watchdog=true;bool startup_motor_calibration=false,startup_encoder_index_search=false,startup_encoder_offset_calibration=false,startup_homing=false,startup_closed_loop_control=false,startup_sensorless_control=false,enable_step_dir=false,step_dir_always_on=false;LockinConfig_t general_lockin,sensorless_ramp;}config_;
  uint32_t error_=0,last_heartbeat_=0,feed_count=0,watchdog_current_value_=3,watchdog_reset_value_=3;
- AxisState current_state_=AXIS_STATE_IDLE,requested_state_=AXIS_STATE_UNDEFINED;
+ std::array<AxisState,10> task_chain_{{AXIS_STATE_IDLE}};AxisState& current_state_=task_chain_.front();AxisState requested_state_=AXIS_STATE_UNDEFINED;uint32_t loop_counter_=0;
+ bool do_checks(){return true;}bool do_updates(){return true;}void update_brake_current(){}void set_step_dir_active(bool){}
+ bool wait_for_current_meas(){error_reads=0;++virtual_tick;if(meas_hook){auto f=meas_hook;f();}return true;}
+ bool run_homing(){return true;}bool run_lockin_spin(const LockinConfig_t&){return true;}bool run_sensorless_control_loop(){return true;}
+ bool run_closed_loop_control_loop();bool run_idle_loop();void run_state_machine_loop();
  /* ACTUAL_CAN_AXIS_FIELDS */
  Motor motor_;Controller controller_;Encoder encoder_;SensorlessEstimator sensorless_estimator_;struct{struct{float vel_limit=0,accel_limit=0,decel_limit=0;}config_;}trap_traj_;
  uint32_t get_watchdog_reset(){++feed_count;return watchdog_reset_value_;}
@@ -105,5 +136,6 @@
 };
 constexpr int AXIS_COUNT=2;
 Axis a0,a1;Axis* axes[2]={&a0,&a1};
+void advance_calibration(){for(uint32_t n=0;n<calibration_ms;++n){++virtual_tick;if(meas_hook){auto f=meas_hook;f();}}}
 class ODriveCAN;ODriveCAN* odCAN=nullptr;
 float vbus_voltage=48;
diff --color -ruN a/firmware/mks_odrive/tests/run_source_tests.py b/firmware/mks_odrive/tests/run_source_tests.py
--- a/firmware/mks_odrive/tests/run_source_tests.py
+++ b/firmware/mks_odrive/tests/run_source_tests.py
@@ -23,6 +23,21 @@
     return re.sub(r'^\s*#(?:include|pragma once)[^\n]*', '', path.read_text(), flags=re.M)
 
 
+DEFINE = re.compile(r'^#define\s+((?:CAN|HAL_CAN_ERROR)_\w+)\s+(.+?)\s*(?:/\*.*)?$')
+
+
+def real_can_defines(root):
+    sources = [root / 'Board/v3/Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f405xx.h',
+               root / 'Board/v3/Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_can.h']
+    lines = []
+    for path in sources:
+        for line in path.read_text(errors='replace').splitlines():
+            match = DEFINE.match(line)
+            if match and not line.rstrip().endswith('\\'):
+                lines.append(f'#define {match.group(1)} {match.group(2)}')
+    return '\n'.join(lines)
+
+
 def main():
     ap = argparse.ArgumentParser()
     ap.add_argument('source', type=Path, help='Extracted ODrive-fw-v0.5.1 root or Firmware directory')
@@ -32,14 +47,16 @@
     interface = root / 'communication/interface_can.hpp'
     patched = 'reliability_patch_' in interface.read_text()
     axis_header = (root / 'MotorControl/axis.hpp').read_text()
-    axis_methods = function(axis_header, 'void clear_errors()') + '\n' + function(axis_header, 'bool inline check_for_errors()')
+    axis_methods = 'template<typename T>\n' + function(axis_header, 'void run_control_loop(const T& update_handler) {') + '\n' + function(axis_header, 'void clear_errors()') + '\n' + function(axis_header, 'bool inline check_for_errors()')
     if 'void latch_can_fault(' in axis_header:
         axis_methods += '\n' + function(axis_header, 'void latch_can_fault(')
-    prelude = (local / 'hal_boundary.hpp').read_text().replace('/* ACTUAL_AXIS_METHODS */', axis_methods)
+    prelude = ((local / 'hal_boundary.hpp').read_text().replace('/* REAL_CAN_DEFINES */', real_can_defines(root)).replace('/* ACTUAL_AXIS_METHODS */', axis_methods))
     fields = '\n'.join(re.findall(r'^\s*(?:bool|uint8_t) can_recovery_\w+_ = [^;]+;', axis_header, re.M))
     prelude = prelude.replace('/* ACTUAL_CAN_AXIS_FIELDS */', fields or
                             'bool can_recovery_latched_=false, can_recovery_in_progress_=false;')
     hal = function((root / 'Board/v3/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c').read_text(), 'HAL_StatusTypeDef HAL_CAN_AddTxMessage(')
+    hal_c = (root / 'Board/v3/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c').read_text()
+    irq_handler = function(hal_c, 'void HAL_CAN_IRQHandler(CAN_HandleTypeDef *hcan)')
     can_source = stripped(root / 'communication/interface_can.cpp')
     # Observe the actual encoder's packet at the TX boundary, including gated
     # attempts during reset. The production write body still executes unchanged.
@@ -48,15 +65,20 @@
     can_source = can_source.replace(signature, signature + '\n    observe_tx_attempt(txmsg);')
     source = '\n'.join([
         '#define PATCHED ' + str(int(patched)),
+        '#define PATCH4 ' + str(int('take_informational_errors' in interface.read_text())),
         '#define RECOVERY_METADATA ' + str(int('can_recovery_auto_resume_eligible_' in axis_header)), prelude,
         stripped(root / 'communication/can_helpers.hpp'),
         stripped(interface).replace('private:', 'public:'),
         stripped(root / 'communication/can_simple.hpp'), hal,
         function((root / 'MotorControl/axis.cpp').read_text(), 'void Axis::watchdog_feed()'),
         function((root / 'MotorControl/axis.cpp').read_text(), 'bool Axis::watchdog_check()'),
+        function((root / 'MotorControl/axis.cpp').read_text(), 'bool Axis::run_closed_loop_control_loop() {'),
+        function((root / 'MotorControl/axis.cpp').read_text(), 'bool Axis::run_idle_loop() {'),
+        function((root / 'MotorControl/axis.cpp').read_text(), 'void Axis::run_state_machine_loop() {'),
         function((root / 'MotorControl/low_level.cpp').read_text(), 'void safety_critical_arm_motor_pwm('),
         function((root / 'MotorControl/low_level.cpp').read_text(), 'bool safety_critical_disarm_motor_pwm('),
         can_source,
+        irq_handler,
         stripped(root / 'communication/can_simple.cpp'),
         (local / 'scenarios.cpp').read_text(),
     ])
@@ -74,6 +96,7 @@
               'resume_generation_wrap', 'resume_reboot_defaults', 'resume_invalid_baud',
               'resume_patch_identity']
     cases += ['guard_resume_success', 'guard_resume_generation', 'guard_resume_new_axis_error', 'guard_resume_estop', 'guard_resume_manual_reinit', 'guard_resume_pending_hal', 'guard_resume_hal_error', 'guard_resume_reinitializing', 'guard_resume_progress', 'guard_resume_nonidle', 'guard_resume_pending_arm', 'guard_resume_motor_error', 'guard_resume_encoder_error', 'guard_resume_controller_error', 'guard_resume_sensorless_error', 'guard_resume_not_latched', 'guard_resume_ineligible', 'guard_resume_external_clear', 'guard_resume_wrong_magic', 'guard_resume_reserved', 'guard_resume_wrong_state', 'guard_resume_invalid_frame', 'guard_resume_pending_idle', 'guard_resume_atomic', 'guard_resume_replay_after_fault', 'guard_resume_late_fault', 'guard_manual_zero', 'guard_manual_empty', 'guard_clear_nonzero', 'guard_clear_old_cas']
+    cases += ['harness_irq_model', 'harness_state_machine']
     cases += ['guard_resume_reserved_eligible', 'guard_resume_new_generation', 'guard_resume_no_can', 'guard_manual_state4', 'guard_manual_state8']
     with tempfile.TemporaryDirectory(prefix='mks-source-tests-') as temp:
         cpp, exe = Path(temp) / 'source.cpp', Path(temp) / 'source-tests'
diff --color -ruN a/firmware/mks_odrive/tests/scenarios.cpp b/firmware/mks_odrive/tests/scenarios.cpp
--- a/firmware/mks_odrive/tests/scenarios.cpp
+++ b/firmware/mks_odrive/tests/scenarios.cpp
@@ -14,7 +14,7 @@
 int main(int argc,char** argv){try{
  require(argc==2,"case required");std::string name=argv[1];
  CAN_HandleTypeDef h;ODriveCAN::Config_t cfg;ODriveCAN can(cfg,&h);odCAN=&can;a1.config_.can_node_id=14;can.thread_id_valid_=true;
- for(auto axis:axes){axis->motor_.axis_=axis;axis->motor_.hw_config_.timer=&axis->motor_;}
+ for(auto axis:axes){axis->motor_.axis_=axis;axis->motor_.hw_config_.timer=&axis->motor_;axis->controller_.pos_estimate_linear_src_=&axis->encoder_.pos_estimate_;}
  can_Message_t reply;
  if(name.rfind("guard_",0)==0){
 #if RECOVERY_METADATA
@@ -251,6 +251,25 @@
 #else
   require(false,"no retained Stop error diagnostic");
 #endif
+ }else if(name=="harness_irq_model"){
+  irq_handle=&h;can.thread_id_valid_=false;can.start_can_server();
+  regs.IER|=CAN_IT_RX_FIFO0_OVERRUN|CAN_IT_ERROR|CAN_IT_LAST_ERROR_CODE;
+  for(int n=0;n<4;++n)deliver((13<<5)|9,false,true);
+  require(fifo[0]==3 && (regs.RF0R&CAN_RF0R_FOVR0),"FIFO model did not overrun at depth 3");
+  fire_can_irq();
+  require(!(regs.RF0R&CAN_RF0R_FOVR0),"real IRQ handler did not clear FOVR0");
+  regs.IER|=CAN_IT_ERROR|CAN_IT_LAST_ERROR_CODE;inject_esr(1u<<CAN_ESR_LEC_Pos);fire_can_irq();
+  require(!(regs.ESR&CAN_ESR_LEC) && !(regs.MSR&CAN_MSR_ERRI),"real IRQ handler did not consume LEC/ERRI");
+ }else if(name=="harness_state_machine"){
+  a0.config_.enable_watchdog=false;uint32_t entered=0,left=0;
+  meas_hook=[&](){
+   if(virtual_tick==1010){auto m=msg(7,false,4);std::memset(m.buf,0,8);uint32_t s=8;std::memcpy(m.buf,&s,4);CANSimple::handle_can_message(m);}
+   if(!entered && a0.current_state_==Axis::AXIS_STATE_CLOSED_LOOP_CONTROL)entered=virtual_tick;
+   if(virtual_tick==1100)a0.requested_state_=Axis::AXIS_STATE_IDLE;
+   if(entered && !left && a0.current_state_==Axis::AXIS_STATE_IDLE)left=virtual_tick;
+   if(virtual_tick>=1200)throw StopLoop{};};
+  virtual_tick=1000;try{a0.run_state_machine_loop();}catch(const StopLoop&){}meas_hook=nullptr;
+  require(entered>1010 && left>1100 && a0.motor_.armed_state_==Motor::ARMED_STATE_DISARMED,"real dispatcher did not enter and leave CLOSED_LOOP with disarm");
  }else throw std::runtime_error("unknown scenario");
  require(irq_mask==0,"critical section leaked");std::cout<<"ok\n";return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}}
```

변경 요지(검토자용):
- 가짜 CAN 상수를 지우고 실제 헤더 상수를 주입한다. 값이 바뀐 것은 `CAN_IT_*`·`CAN_FILTERSCALE_32BIT` 등 기존 시험이 이름으로만 쓰던 것뿐이다.
- `ActivateNotification`/`DeactivateNotification`이 `IER`를 실제로 켜고 끈다. 쓰기-1-클리어 플래그는 `can_clear_flag`로 모형화한다.
- `rcc_force_reset`은 FIFO 내용과 RF0R/RF1R/ESR/MSR/IER도 지운다.
- 축 스텁에 실제 상태머신이 쓰는 필드와 스텁(`do_checks`·`do_updates`·캘리 등)을 더한다. `wait_for_current_meas`는 tick을 1 올리고 `meas_hook`을 부르며, 이 축 대기에서도 busy-spin 감지기를 리셋한다.
- 시험 초기화에서 `controller_.pos_estimate_linear_src_`를 엔코더 추정값에 연결한다(실제 CLOSED_LOOP 함수의 null 검사 통과).

- [ ] **Step 2: 통과 확인(동작 불변)**

Run: 하네스 실행 3종.
Expected:
- `$WORK/work`, `$WORK/patch3`: `88 passed, 0 failed; PATCHED=True`
- `$WORK/original`: `6 passed, 82 failed; PATCHED=False`

- [ ] **Step 3: 음성 대조** — (a) `hal_boundary.hpp`의 `deliver()`에서 `fifo[0]<3`을 `fifo[0]<4`로 바꾸면 `harness_irq_model`이 `FIFO model did not overrun at depth 3`으로 FAIL. (b) `$WORK/patch3` 사본의 `run_idle_loop` 첫 줄 `safety_critical_disarm_motor_pwm(motor_);`을 지우면 `harness_state_machine`이 FAIL. 각각 확인 후 원복한다.

- [ ] **Step 4: Codex 리뷰(교차검증)** — Task 0·1 diff를 읽기 전용으로 검토받는다.

```bash
node <codex-plugin>/scripts/codex-companion.mjs adversarial-review "Task 0-1 of docs/plans/2026-10-15-mks-can-patch4.md: harness must model real bxCAN IRQ/FIFO/filter and the real axis state machine without changing patch3 results" > /tmp/codex-t1.md 2>&1
```
지적(P1/P2)을 근거로 확인해 반영하고, 같은 스레드(`task --resume-last`, 질문은 파일로 stdin)로 재검토를 요청해 "새 P1/P2 없음"까지 반복한다. 플러그인 경로는 `jq -r '.. | objects | select(has("installPath")) | .installPath' ~/.claude/plugins/installed_plugins.json | grep codex`로 확인한다.

- [ ] **Step 5: 전체 확인**

Run: `MKS_VENDOR_ZIP=$ZIP $PY firmware/mks_odrive/tests/test_prepare.py --zip $ZIP && MKS_VENDOR_ZIP=$ZIP $PY firmware/mks_odrive/tests/test_regen_patch.py && $PY firmware/mks_odrive/tests/test_build_firmware.py`
Expected: 전부 OK(test_prepare 내부 하네스 88/0).

- [ ] **Step 6: 커밋(사용자 확인 후)**

```bash
git add firmware/mks_odrive/regen_patch.py firmware/mks_odrive/tests/test_regen_patch.py \
  firmware/mks_odrive/tests/hal_boundary.hpp firmware/mks_odrive/tests/run_source_tests.py \
  firmware/mks_odrive/tests/scenarios.cpp
git commit -m "test(firmware): MKS 하네스에 실제 CAN IRQ·필터와 축 상태머신을 붙이고 패치 재생성 도구를 더한다"
```

---

### Task 2: D1 오류 분류와 D3 복구 후 대기 제거

**Files:**
- Modify(작업 트리): `$WORK/work/Firmware/communication/interface_can.hpp`, `$WORK/work/Firmware/communication/interface_can.cpp`
- Modify: `firmware/mks_odrive/tests/scenarios.cpp`, `firmware/mks_odrive/tests/run_source_tests.py`
- Regenerate: `firmware/mks_odrive/reliability.patch`

**Interfaces:**
- Produces(펌웨어): `uint32_t ODriveCAN::take_informational_errors()`(정보성 비트를 계수·제거하고 남은 비-정보성 비트를 반환), `void ODriveCAN::request_reinit(bool manual)`, `void ODriveCAN::count_tx_drop()`, 상수 `kInformationalHalErrors=0x7FF`, `kLecHalErrors=0x1F8`, `kRxOverrunHalErrors=0x600`, 계수 `lec_error_count_`, `rx_overrun_count_`, `bus_off_count_`, `error_passive_count_`, `error_warning_count_`.
- `request_reinit`은 Task 4에서 축 표시(in-progress·revoked)를 더한다. 이 Task에서는 `reinit_requested_`/`manual_reinit_requested_` 설정과 세마포어 release만 한다.

- [ ] **Step 1: 실패하는 시험 작성** — `scenarios.cpp`

`main` 위, `guarded_resume` 정의 다음에 추가:

```cpp
void run_one(ODriveCAN& can){waits=0;wait_budget=2;run_server(can);}
```

`harness_irq_model` 분기 다음에 추가:

```cpp
 }else if(name.rfind("info_",0)==0){
#if PATCH4
  irq_handle=&h;can.thread_id_valid_=false;can.start_can_server();
  a0.current_state_=Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;
  if(name=="info_lec_no_latch"){
   for(uint32_t code=1;code<=6;++code){inject_esr(code<<CAN_ESR_LEC_Pos);fire_can_irq();
    require(!can.reinit_requested_,"IRQ requested reinit for a bus error");run_one(can);}
   require(can.lec_error_count_==6 && can.recovery_count_==0 && h.ErrorCode==0,"LEC was not informational");
  }else if(name=="info_overrun_no_latch"){
   for(int n=0;n<4;++n)deliver((13<<5)|9,false,true);
   fire_can_irq();require(!can.reinit_requested_,"IRQ requested reinit for overrun");run_one(can);
   require(can.rx_overrun_count_==1 && can.recovery_count_==0 && rx_reads==3,"overrun was not informational or FIFO not serviced");
  }else if(name=="info_status_no_latch"){
   inject_esr(CAN_ESR_EWGF|CAN_ESR_EPVF|CAN_ESR_BOFF);fire_can_irq();run_one(can);
   require(can.error_warning_count_==1 && can.error_passive_count_==1 && can.bus_off_count_==1 && can.recovery_count_==0,"status event stopped CAN");
  }else if(name=="info_mixed_param"){
   h.ErrorCode=HAL_CAN_ERROR_PARAM|HAL_CAN_ERROR_STF|HAL_CAN_ERROR_FOR|HAL_CAN_ERROR_RX_FOV0;run_one(can);
   require(can.lec_error_count_==1 && can.rx_overrun_count_==1,"mixed informational bits not counted");
   require(can.recovery_count_==1 && (can.hal_error_history_&HAL_CAN_ERROR_PARAM),"API error inside a mixed mask was swallowed");
  }else if(name=="info_state_not_listening"){
   h.State=HAL_CAN_STATE_READY;run_one(can);require(can.recovery_count_==1,"unexpected HAL state not recovered");
  }else if(name=="info_success_then_burst"){
   h.ErrorCode=HAL_CAN_ERROR_PARAM;bool burst=false;
   on_wait=[&](){if(!burst){burst=true;for(int n=0;n<4;++n)deliver((13<<5)|9,false,true);fire_can_irq();}};
   waits=0;wait_budget=3;run_server(can);on_wait=nullptr;
   require(can.recovery_count_==1 && can.rx_overrun_count_==1,"post-recovery burst caused another recovery");
   require(delay_count==0,"successful recovery still sleeps with RX unattended");
  }else if(name=="info_storm"){
   for(uint32_t n=0;n<10000;++n){inject_esr(1u<<CAN_ESR_LEC_Pos);fire_can_irq();if(n%100==99)run_one(can);}
   run_one(can);
   require(can.lec_error_count_>0 && can.recovery_count_==0 && h.ErrorCode==0,"LEC storm stopped CAN or left errors");
   require((regs.IER&CAN_IT_ERROR)!=0,"error notifications not re-enabled after storm");
   const uint32_t before=can.lec_error_count_;inject_esr(1u<<CAN_ESR_LEC_Pos);fire_can_irq();
   require(can.lec_error_count_==before+1,"LEC after the storm was not observed (notifications stayed off)");
  }else throw std::runtime_error("unknown info scenario");
  if(name!="info_mixed_param" && name!="info_state_not_listening" && name!="info_success_then_burst")
   require(a0.error_==0 && !a0.can_recovery_latched_ && !a0.motor_.disarmed,"informational event stopped the motor");
#else
  require(false,"patch4 informational classification missing");
#endif
```

기존 `resume_manual_at_dispatch` 분기는 "두 번째 critical 구간 해제" 횟수에 맞춰 수동 요청을 주입한다. patch4는 서버 루프 맨 위에 `take_informational_errors()`의 critical 구간을 더하므로 횟수 기준이 어긋난다. 의도("래치 직후 dispatch 시점의 수동 요청")를 소스와 무관하게 표현하도록 그 분기의 첫 줄

```cpp
   unsigned releases=0;on_irq_enable=[&](){if(++releases==2){on_irq_enable=nullptr;can.set_baud_rate(500000);}};
```

을 다음으로 바꾼다(patch3에서도 같은 시점에 발화한다):

```cpp
   on_irq_enable=[&](){if(a0.can_recovery_latched_){on_irq_enable=nullptr;can.set_baud_rate(500000);}};
```

`server_error` 분기의 `require(delay_count>0,"error server failed to yield");`를 다음으로 바꾼다(D3: 성공 복구는 대기 없이 RX 서비스로 돌아가고 세마포어 대기로 양보한다):

```cpp
  require(name=="server_retry_failure"?delay_count>0:waits>0,"error server failed to yield");
```

러너 `cases`에 추가:

```python
    cases += ['info_lec_no_latch', 'info_overrun_no_latch', 'info_status_no_latch', 'info_mixed_param',
              'info_state_not_listening', 'info_success_then_burst', 'info_storm']
```

- [ ] **Step 2: 실패 확인**

Run: 하네스 실행(`$WORK/work`).
Expected: 7개 `info_*` FAIL(`patch4 informational classification missing`), 나머지 기존 PASS. 같은 수정 시험을 `$WORK/patch3`에도 돌려 `resume_manual_at_dispatch`·`server_error`가 여전히 PASS인지 확인한다(트리거 변경이 의미를 바꾸지 않았다는 증거).

- [ ] **Step 3: 구현 — `interface_can.hpp`** (`$WORK/work/Firmware/communication/interface_can.hpp`)

`const uint32_t reliability_patch_ = 3;`부터 `void record_hal_error(...);`까지를 다음으로 바꾼다(식별자 3→4는 Task 7에서 바꾼다):

```cpp
    // RAM-only diagnostics: Config_t (and therefore NVM layout) is unchanged.
    const uint32_t reliability_patch_ = 3;
    // Bus/FIFO events the protocol already handles: counted, never a stop (patch4 D1).
    static constexpr uint32_t kInformationalHalErrors = 0x000007FFu;  // EWG..RX_FOV1
    static constexpr uint32_t kLecHalErrors = 0x000001F8u;            // STF..CRC
    static constexpr uint32_t kRxOverrunHalErrors = 0x00000600u;      // RX_FOV0|RX_FOV1
    uint32_t last_hal_error_ = 0;
    uint32_t hal_error_history_ = 0;
    uint32_t tx_drop_count_ = 0;
    uint32_t tx_error_count_ = 0;
    uint32_t recovery_count_ = 0;
    uint32_t recovery_failure_count_ = 0;
    uint32_t rx_discard_count_ = 0;
    uint32_t lec_error_count_ = 0;
    uint32_t rx_overrun_count_ = 0;
    uint32_t bus_off_count_ = 0;
    uint32_t error_passive_count_ = 0;
    uint32_t error_warning_count_ = 0;
    bool reinitializing_ = false;
    bool reinit_requested_ = false;
    uint32_t get_hal_error();
    void record_hal_error(uint32_t error, bool request_reinit = true);
    uint32_t take_informational_errors();
    void request_reinit(bool manual);
    void count_tx_drop();
```

- [ ] **Step 4: 구현 — `interface_can.cpp`**

`record_hal_error`를 교체하고 그 아래 세 함수를 추가:

```cpp
void ODriveCAN::record_hal_error(uint32_t error, bool request) {
    if (!error) return;
    uint32_t mask = cpu_enter_critical();
    last_hal_error_ = error;
    hal_error_history_ |= error;
    cpu_exit_critical(mask);
    if (request) request_reinit(false);
}

uint32_t ODriveCAN::take_informational_errors() {
    // Read, count and clear only the informational bits in one critical
    // section so an IRQ cannot OR a new bit between the read and the clear.
    uint32_t mask = cpu_enter_critical();
    const uint32_t observed = HAL_CAN_GetError(handle_);
    if (observed) {
        last_hal_error_ = observed;
        hal_error_history_ |= observed;
    }
    const uint32_t info = observed & kInformationalHalErrors;
    if (info) {
        if (info & kLecHalErrors) ++lec_error_count_;
        if (info & kRxOverrunHalErrors) ++rx_overrun_count_;
        if (info & HAL_CAN_ERROR_BOF) ++bus_off_count_;
        if (info & HAL_CAN_ERROR_EPV) ++error_passive_count_;
        if (info & HAL_CAN_ERROR_EWG) ++error_warning_count_;
        handle_->ErrorCode = observed & ~info;
    }
    cpu_exit_critical(mask);
    return observed & ~info;
}

void ODriveCAN::request_reinit(bool manual) {
    uint32_t mask = cpu_enter_critical();
    reinit_requested_ = true;
    if (manual) manual_reinit_requested_ = true;
    cpu_exit_critical(mask);
    osSemaphoreRelease(sem_can);  // never under PRIMASK
}

void ODriveCAN::count_tx_drop() {
    uint32_t mask = cpu_enter_critical();
    ++tx_drop_count_;
    cpu_exit_critical(mask);
}
```

`can_server_thread()` 전체를 다음으로 교체:

```cpp
void ODriveCAN::can_server_thread() {
    for (;;) {
        // Informational events never stop motors. Only API-class errors or an
        // unexpected HAL state request a reinitialization (patch4 D1).
        if (take_informational_errors() != HAL_CAN_ERROR_NONE ||
            (!reinit_requested_ && !reinitializing_ && handle_->State != HAL_CAN_STATE_LISTENING))
            request_reinit(false);
        if (reinit_requested_ || reinitializing_) {
            uint32_t mask = cpu_enter_critical();
            reinitializing_ = true; // All TX/RX entry points reject until complete.
            reinit_requested_ = false;
            const uint32_t baud = pending_baud_rate_;
            const bool automatic = !manual_reinit_requested_;
            manual_reinit_requested_ = false;
            // Capture provenance and disarm atomically with the queued origin.
            for (auto axis : axes) axis->latch_can_fault(automatic);
            cpu_exit_critical(mask);
            // No blocking HAL call is made under PRIMASK. Both motors are stopped.
            ++recovery_count_;
            bool ok = apply_can_config(baud, true);
            mask = cpu_enter_critical();
            if (ok && !reinit_requested_) {
                reinitializing_ = false;
                for (auto axis : axes) axis->can_recovery_in_progress_ = false;
            }
            if (!ok) ++recovery_failure_count_;
            cpu_exit_critical(mask);
            if (!ok) {
                // Persistent faults must never starve idle/USB.
                osDelay(10);
                continue;
            }
            // Success returns to RX service at once (patch4 D3): no unattended
            // window in which a burst can overrun the three-deep FIFO.
        }

        osSemaphoreWait(sem_can, 10);
        // A saturated bus cannot create an unbounded RX loop.
        for (unsigned int n = 0; n < 16; ++n) {
            can_Message_t rxmsg;
            if (!read(rxmsg)) break;
            switch (config_.protocol) {
                case PROTOCOL_SIMPLE: CANSimple::handle_can_message(rxmsg); break;
            }
        }
        if (!reinit_requested_ && !reinitializing_) {
            if (HAL_CAN_ActivateNotification(handle_, notifications) != HAL_OK)
                record_hal_error(HAL_CAN_GetError(handle_));
        }
    }
}
```

`write()`의 critical 구간을 다음으로 교체(정보성 비트로 송신을 막지 않고, PRIMASK 안에서는 재초기화 요청을 하지 않는다):

```cpp
uint32_t ODriveCAN::write(can_Message_t &txmsg) {
    CAN_TxHeaderTypeDef header = {};
    header.StdId = txmsg.id;
    header.ExtId = txmsg.id;
    header.IDE = txmsg.isExt ? CAN_ID_EXT : CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = txmsg.len;
    header.TransmitGlobalTime = DISABLE;
    uint32_t mailbox = 0;
    bool request = false;
    uint32_t observed = 0;
    // Only finite, nonblocking register operations are protected. This excludes
    // axis heartbeat preemption between the slot check and HAL registration.
    uint32_t mask = cpu_enter_critical();
    if (reinitializing_ || reinit_requested_ || txmsg.len > 8) {
        ++tx_drop_count_;
    } else if (HAL_CAN_GetError(handle_) & ~kInformationalHalErrors) {
        ++tx_error_count_;
        request = true;
        observed = HAL_CAN_GetError(handle_);
    } else if (!HAL_CAN_GetTxMailboxesFreeLevel(handle_)) {
        ++tx_drop_count_;
    } else if (HAL_CAN_AddTxMessage(handle_, &header, txmsg.buf, &mailbox) != HAL_OK) {
        mailbox = 0;
        ++tx_error_count_;
        request = true;  // Even a HAL failure without an error flag.
        observed = HAL_CAN_GetError(handle_);
    }
    cpu_exit_critical(mask);
    if (request) {
        record_hal_error(observed, false);
        request_reinit(false);
    }
    return mailbox;
}
```

`read()`를 다음으로 교체:

```cpp
bool ODriveCAN::read(can_Message_t &rxmsg) {
    CAN_RxHeaderTypeDef header = {};
    bool valid = false, request = false;
    uint32_t observed = 0;
    uint32_t mask = cpu_enter_critical();
    if (!reinitializing_ && !reinit_requested_) {
        if (HAL_CAN_GetError(handle_) & ~kInformationalHalErrors) {
            request = true;
            observed = HAL_CAN_GetError(handle_);
        } else {
            uint32_t fifo = HAL_CAN_GetRxFifoFillLevel(handle_, CAN_RX_FIFO0) ? CAN_RX_FIFO0 : CAN_RX_FIFO1;
            if (HAL_CAN_GetRxFifoFillLevel(handle_, fifo)) {
                valid = HAL_CAN_GetRxMessage(handle_, fifo, &header, rxmsg.buf) == HAL_OK;
                if (!valid) {
                    request = true;
                    observed = HAL_CAN_GetError(handle_);
                }
            }
        }
    }
    cpu_exit_critical(mask);
    if (request) {
        record_hal_error(observed, false);
        request_reinit(false);
    }
    if (!valid) return false;
    rxmsg.isExt = header.IDE == CAN_ID_EXT;
    rxmsg.id = rxmsg.isExt ? header.ExtId : header.StdId;
    rxmsg.len = header.DLC;
    rxmsg.rtr = header.RTR != CAN_RTR_DATA;
    return true;
}
```

`set_baud_rate()`와 `reinit_can()`을 다음으로 교체:

```cpp
void ODriveCAN::set_baud_rate(uint32_t baudRate) {
    if (baudRate != CAN_BAUD_125K && baudRate != CAN_BAUD_250K &&
        baudRate != CAN_BAUD_500K && baudRate != CAN_BAUD_1000K) return;
    // Fibre/USB callers only enqueue. The server owns every runtime HAL reset.
    // Multiple requests coalesce to the most recently requested valid baud.
    uint32_t mask = cpu_enter_critical();
    pending_baud_rate_ = baudRate;
    for (auto axis : axes) axis->can_recovery_auto_resume_eligible_ = false;
    cpu_exit_critical(mask);
    request_reinit(true);
}

void ODriveCAN::reinit_can() {
    uint32_t mask = cpu_enter_critical();
    for (auto axis : axes) axis->can_recovery_auto_resume_eligible_ = false;
    cpu_exit_critical(mask);
    request_reinit(true);
}
```

`HAL_CAN_ErrorCallback`을 다음으로 교체:

```cpp
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
    // Informational bus/FIFO events are counted and cleared here. API-class
    // errors stay in ErrorCode for the server; IRQ context never reinitializes.
    if (odCAN) odCAN->take_informational_errors();
    HAL_CAN_DeactivateNotification(hcan, notifications & ~CAN_IT_RX_FIFO0_MSG_PENDING);
    osSemaphoreRelease(sem_can);
}
```

- [ ] **Step 5: 통과 확인**

Run: 하네스 실행(`$WORK/work`).
Expected: `95 passed, 0 failed`. `$WORK/patch3`: `info_*` 7개 FAIL(`88 passed, 7 failed`).

기존 시험 중 실패가 나면 원인을 기록한다. 특히 `irq_error_retained`는 PARAM이 정보성이 아니므로 계속 PASS여야 한다.

- [ ] **Step 6: 음성 대조** — `kInformationalHalErrors`를 `0x000001FFu`(RX_FOV 제외)로 임시 변경 → `info_overrun_no_latch`, `info_success_then_burst` FAIL 확인 후 원복. `can_server_thread`에서 성공 경로에 `osDelay(10);`을 임시로 넣어 `info_success_then_burst` FAIL 확인 후 원복.

- [ ] **Step 7: 패치 재생성과 재현 확인**

```bash
$PY firmware/mks_odrive/regen_patch.py --original $WORK/original --work $WORK/work
MKS_VENDOR_ZIP=$ZIP $PY firmware/mks_odrive/tests/test_regen_patch.py
```
`test_prepare.py`는 식별자 기대값(3)이 Task 7에서 바뀌므로 이 단계에서는 하네스 실행 결과만 확인한다.

- [ ] **Step 8: 커밋 보류** — Task 3과 함께 커밋(Task 3 Step 9).

---

### Task 3: D2 수신 필터와 송신 ID 범위 검사

**Files:**
- Modify(작업 트리): `$WORK/work/Firmware/communication/interface_can.hpp`, `interface_can.cpp`, `can_simple.cpp`
- Modify: `firmware/mks_odrive/tests/scenarios.cpp`, `firmware/mks_odrive/tests/run_source_tests.py`
- Regenerate: `firmware/mks_odrive/reliability.patch`

**Interfaces:**
- Consumes: `ODriveCAN::count_tx_drop()`, `request_reinit(bool)`(Task 2).
- Produces(펌웨어): `bool ODriveCAN::program_rx_filters()`, `void ODriveCAN::update_rx_filters_if_needed()`, can_simple.cpp 파일 범위 `static bool make_tx_id(Axis*, uint32_t cmd, can_Message_t&)`.

- [ ] **Step 1: 실패하는 시험 작성** — `scenarios.cpp`, `info_` 분기 다음:

```cpp
 }else if(name.rfind("filter_",0)==0 || name.rfind("txid_",0)==0){
#if PATCH4
  irq_handle=&h;
  if(name=="filter_ext_node"){a0.config_.can_node_id=0x40;a0.config_.can_node_id_extended=true;}
  if(name=="filter_out_of_range")a0.config_.can_node_id=77;
  can.thread_id_valid_=false;can.start_can_server();
  if(name=="filter_own_nodes"){
   require(deliver((13<<5)|13) && deliver((14<<5)|9,false,true),"own node frames rejected");
   require(!deliver((15<<5)|13) && !deliver((12<<5)|9,false,true),"foreign node accepted");
   require(!deliver((13<<5)|13,true) && !deliver(0x00000301,true),"extended or AK frame accepted by STD filter");
   require(fifo[0]==2,"FIFO holds foreign traffic");
   for(uint32_t cmd=0;cmd<32;++cmd)for(bool rtr:{false,true}){
    require(deliver((13<<5)|cmd,false,rtr) && deliver((14<<5)|cmd,false,rtr),"own node command rejected by filter");
    require(!deliver((15<<5)|cmd,false,rtr) && !deliver((13<<5)|cmd,true,rtr),"foreign node or EXT frame accepted");}
  }else if(name=="filter_ext_node"){
   require(deliver((0x40<<5)|9,true),"extended own node rejected");
   require(!deliver(((0x40<<5)|9)&0x7FF) && !deliver((13<<5)|9),"STD alias or old node accepted");
  }else if(name=="filter_out_of_range"){
   require(!filters[0].FilterActivation,"out-of-range node opened a filter bank");
   require(!deliver(((77<<5)|9)&0x7FF),"out-of-range node aliased into node13");
   require(deliver((14<<5)|9),"in-range peer axis lost its filter");
  }else if(name=="filter_can_node_change" || name=="filter_usb_node_change"){
   const uint32_t calls=filter_config_calls;
   if(name=="filter_can_node_change"){auto m=msg(6,false,4);std::memset(m.buf,0,8);uint32_t node=21;std::memcpy(m.buf,&node,4);CANSimple::handle_can_message(m);}
   else a0.config_.can_node_id=21;
   run_one(can);
   require(filter_config_calls>calls && deliver((21<<5)|9) && !deliver((13<<5)|9),"node change did not reprogram the filter");
  }else if(name=="filter_config_failure"){
   fail_filter=true;a0.config_.can_node_id=21;waits=0;wait_budget=3;run_server(can);
   require(can.recovery_count_>=1 && a0.can_recovery_latched_,"filter programming failure was not treated as a HAL failure");
  }else if(name=="txid_std_range" || name=="txid_ext_range"){
   const bool ext=name=="txid_ext_range";
   auto send_all=[&](){can_Message_t q=msg(9,true);CANSimple::send_heartbeat(&a0);
    CANSimple::get_motor_error_callback(&a0,q);CANSimple::get_encoder_error_callback(&a0,q);
    CANSimple::get_sensorless_error_callback(&a0,q);CANSimple::get_encoder_estimates_callback(&a0,q);
    CANSimple::get_sensorless_estimates_callback(&a0,q);CANSimple::get_encoder_count_callback(&a0,q);
    CANSimple::get_iq_callback(&a0,q);CANSimple::get_vbus_voltage_callback(&a0,q);};
   std::vector<uint32_t> rejected=ext?std::vector<uint32_t>{0x0800000Du}:std::vector<uint32_t>{0x40u,77u,0x0800000Du};
   for(uint32_t node:rejected){
    a0.config_.can_node_id=node;a0.config_.can_node_id_extended=ext;
    const uint32_t drops=can.tx_drop_count_;observed_tx_len=0xff;send_all();
    require(observed_tx_len==0xff,"out-of-range node reached CAN write");
    require(can.tx_drop_count_==drops+9,"each out-of-range TX site must count one drop");
   }
   if(ext)for(uint32_t node:{0x40u,77u}){
    a0.config_.can_node_id=node;a0.config_.can_node_id_extended=true;observed_tx_len=0xff;
    CANSimple::send_heartbeat(&a0);
    require(observed_tx_len==8 && observed_tx_id==((node<<5)|1),"valid extended node blocked or mis-encoded");
   }
  }else if(name=="txid_write_limit"){
   can_Message_t m;m.isExt=false;m.id=0x800;const uint32_t drops=can.tx_drop_count_;const uint32_t tsr=regs.TSR;
   require(can.write(m)==0 && can.tx_drop_count_==drops+1 && regs.TSR==tsr,"STD ID above 0x7FF reached HAL");
   m.isExt=true;m.id=0x20000000;require(can.write(m)==0 && can.tx_drop_count_==drops+2,"EXT ID above 29 bits reached HAL");
   m.id=0x1FFFFFFF;require(can.write(m)!=0,"maximum valid EXT ID rejected");
  }else throw std::runtime_error("unknown filter/txid scenario");
#else
  require(false,"patch4 node filter/TX range missing");
#endif
```

러너에서 `CANSimple`의 private 콜백을 시험이 직접 부를 수 있게, `stripped(root / 'communication/can_simple.hpp'),` 줄을 다음으로 바꾼다(하네스 안에서만의 접근성 변환이며 펌웨어는 바꾸지 않는다):

```python
        stripped(root / 'communication/can_simple.hpp').replace('private:', 'public:'),
```

러너 `cases`에 추가:

```python
    cases += ['filter_own_nodes', 'filter_ext_node', 'filter_out_of_range', 'filter_can_node_change',
              'filter_usb_node_change', 'filter_config_failure',
              'txid_std_range', 'txid_ext_range', 'txid_write_limit']
```

- [ ] **Step 2: 실패 확인** — 하네스 실행(`$WORK/work`). Expected: 새 9개 FAIL, 나머지 95 PASS.

- [ ] **Step 3: 구현 — `interface_can.hpp`**: `private:` 구역 끝(`bool manual_reinit_requested_ = false;` 다음)에 추가

```cpp
    bool program_rx_filters();
    void update_rx_filters_if_needed();
    uint32_t filter_node_[AXIS_COUNT] = {};
    bool filter_ext_[AXIS_COUNT] = {};
    bool filters_programmed_ = false;
```

- [ ] **Step 4: 구현 — `interface_can.cpp`**

`apply_can_config()`의 필터 블록(`CAN_FilterTypeDef filter = {};`부터 `filter.SlaveStartFilterBank = 14;`까지)과 그 뒤 `if (HAL_CAN_ConfigFilter(handle_, &filter) != HAL_OK ||` 조건을 다음으로 바꾼다:

```cpp
    if (!program_rx_filters() ||
        HAL_CAN_Start(handle_) != HAL_OK ||
        HAL_CAN_ActivateNotification(handle_, notifications) != HAL_OK) {
        record_hal_error(HAL_CAN_GetError(handle_), false);
        return false;
    }
```

같은 함수의 `HAL_CAN_Init` 실패 분기 `record_hal_error(HAL_CAN_GetError(handle_));`도 `record_hal_error(HAL_CAN_GetError(handle_), false);`로 바꾼다. 재시도는 서버의 실패 경로(`ok == false` → `osDelay(10)` → 재시도)와 기동 경로(`start_can_server`의 `reinitializing_ = !ok`)가 이미 소유하므로, 이 함수 안에서 별도 재초기화를 요청하지 않는다.

파일 끝 `set_error` 위에 추가:

```cpp
bool ODriveCAN::program_rx_filters() {
    // Hardware accepts only this board's two nodes (patch4 D2). RTR is
    // don't-care (queries), IDE is compared, the 5 command bits are ignored.
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        const uint32_t node = axes[i]->config_.can_node_id;
        const bool ext = axes[i]->config_.can_node_id_extended;
        const bool in_range = node <= (ext ? 0xFFFFFFu : 0x3Fu);
        const uint32_t id = !in_range ? 0u : ext ? ((node << 8) | 4u) : (node << 26);
        const uint32_t id_mask = ext ? 0xFFFFFF04u : 0xFC000004u;
        CAN_FilterTypeDef filter = {};
        filter.FilterBank = i;
        filter.FilterActivation = in_range ? ENABLE : DISABLE;  // out of range: no bank
        filter.FilterFIFOAssignment = CAN_RX_FIFO0;
        filter.FilterMode = CAN_FILTERMODE_IDMASK;
        filter.FilterScale = CAN_FILTERSCALE_32BIT;
        filter.FilterIdHigh = id >> 16;
        filter.FilterIdLow = id & 0xFFFFu;
        filter.FilterMaskIdHigh = id_mask >> 16;
        filter.FilterMaskIdLow = id_mask & 0xFFFFu;
        filter.SlaveStartFilterBank = 14;
        if (HAL_CAN_ConfigFilter(handle_, &filter) != HAL_OK) return false;
        filter_node_[i] = node;
        filter_ext_[i] = ext;
    }
    filters_programmed_ = true;
    return true;
}

void ODriveCAN::update_rx_filters_if_needed() {
    bool changed = !filters_programmed_;
    for (size_t i = 0; i < AXIS_COUNT; ++i)
        changed |= filter_node_[i] != axes[i]->config_.can_node_id ||
                   filter_ext_[i] != axes[i]->config_.can_node_id_extended;
    if (changed && !program_rx_filters()) {
        record_hal_error(HAL_CAN_GetError(handle_), false);
        request_reinit(false);
    }
}
```

`can_server_thread()`에서 `osSemaphoreWait(sem_can, 10);` 바로 앞에 한 줄 추가:

```cpp
        update_rx_filters_if_needed();
```

`write()`의 첫 조건을 다음으로 바꾼다:

```cpp
    const uint32_t id_limit = txmsg.isExt ? 0x1FFFFFFFu : 0x7FFu;
    if (reinitializing_ || reinit_requested_ || txmsg.len > 8 || txmsg.id > id_limit) {
```

(`id_limit` 선언은 `uint32_t mask = cpu_enter_critical();` 앞에 둔다.)

- [ ] **Step 5: 구현 — `can_simple.cpp`**

`static constexpr uint8_t NUM_CMD_ID_BITS = ...;` 다음에 추가:

```cpp
// Never let a wide node value wrap into another node's 11/29-bit identifier
// (patch4 D2): check the node before shifting, at every TX construction site.
static bool make_tx_id(Axis* axis, uint32_t cmd, can_Message_t& txmsg) {
    const uint32_t node = axis->config_.can_node_id;
    const bool ext = axis->config_.can_node_id_extended;
    if (node > (ext ? 0xFFFFFFu : 0x3Fu)) {
        if (odCAN) odCAN->count_tx_drop();
        return false;
    }
    txmsg.id = (node << NUM_CMD_ID_BITS) + cmd;
    txmsg.isExt = ext;
    return true;
}
```

다음 9곳 각각의 세 줄

```cpp
        txmsg.id = axis->config_.can_node_id << NUM_CMD_ID_BITS;
        txmsg.id += MSG_XXX;  // (주석 유무 무관)
        txmsg.isExt = axis->config_.can_node_id_extended;
```

을 `if (!make_tx_id(axis, MSG_XXX, txmsg)) return;` 한 줄로 바꾼다(들여쓰기는 원래 위치 유지). 대상과 `MSG_XXX`:

| 함수 | MSG |
|---|---|
| `get_motor_error_callback` | `MSG_GET_MOTOR_ERROR` |
| `get_encoder_error_callback` | `MSG_GET_ENCODER_ERROR` |
| `get_sensorless_error_callback` | `MSG_GET_SENSORLESS_ERROR` |
| `get_encoder_estimates_callback` | `MSG_GET_ENCODER_ESTIMATES` |
| `get_sensorless_estimates_callback` | `MSG_GET_SENSORLESS_ESTIMATES` |
| `get_encoder_count_callback` | `MSG_GET_ENCODER_COUNT` |
| `get_iq_callback` | `MSG_GET_IQ` |
| `get_vbus_voltage_callback` | `MSG_GET_VBUS_VOLTAGE` |
| `send_heartbeat` | `MSG_ODRIVE_HEARTBEAT` |

확인: `grep -c "can_node_id << NUM_CMD_ID_BITS" $WORK/work/Firmware/communication/can_simple.cpp` → `0`, `grep -c "make_tx_id(axis," ...` → `9`.

- [ ] **Step 6: 통과 확인** — 하네스 실행(`$WORK/work`): `104 passed, 0 failed`. `$WORK/patch3`: 새 9개 포함 16 FAIL.

- [ ] **Step 7: 음성 대조** — (a) `make_tx_id`의 범위 검사를 지우고 `write()` 검사만 남기면 `txid_std_range`(node `0x0800000D`)가 FAIL해야 한다. (b) STD mask를 `0xFC000000u`(IDE 무시)로 바꾸면 `filter_own_nodes`가 FAIL해야 한다. 각각 확인 후 원복.

- [ ] **Step 8: Codex 리뷰(교차검증)** — Task 2·3의 작업 트리 diff(`diff -ru $WORK/patch3/Firmware $WORK/work/Firmware`)와 시험 diff를 파일로 만들어 `task`(stdin)로 보낸다. 질문 파일에 설계 문서 D1·D2·D3 경로, 시험 결과, "무엇이 깨지는가"를 적는다. 새 P1/P2가 없을 때까지 `task --resume-last`로 반복한다.

- [ ] **Step 9: 패치 재생성·커밋(사용자 확인 후)**

```bash
$PY firmware/mks_odrive/regen_patch.py --original $WORK/original --work $WORK/work
MKS_VENDOR_ZIP=$ZIP $PY firmware/mks_odrive/tests/test_regen_patch.py
git add firmware/mks_odrive/reliability.patch firmware/mks_odrive/tests/scenarios.cpp firmware/mks_odrive/tests/run_source_tests.py
git commit -m "fix(firmware): 정보성 CAN 오류를 정지 대신 계수하고 자기 node만 수신·송신한다"
```

---

### Task 4: D5 재초기화 요청 표시·clear 거절·arm 차단

**Files:**
- Modify(작업 트리): `$WORK/work/Firmware/communication/interface_can.cpp`, `$WORK/work/Firmware/MotorControl/axis.hpp`, `$WORK/work/Firmware/MotorControl/low_level.cpp`
- Modify: `firmware/mks_odrive/tests/scenarios.cpp`, `firmware/mks_odrive/tests/run_source_tests.py`

**Interfaces:**
- Produces(펌웨어): 축 필드 `bool can_recovery_episode_revoked_ = false;`(Task 5가 사용), `request_reinit`의 축 표시, `clear_errors()` 새 거절 규칙.

- [ ] **Step 1: 실패하는 시험 작성** — `scenarios.cpp`, `filter_/txid_` 분기 다음:

```cpp
 }else if(name.rfind("pend_",0)==0){
#if PATCH4
  a0.current_state_=Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;
  if(name=="pend_clear_before_latch"){
   can.record_hal_error(HAL_CAN_ERROR_PARAM);
   require(a0.can_recovery_in_progress_ && !a0.can_recovery_latched_,"request did not mark the axes in progress");
   a0.encoder_.spi_error_rate_=0.5f;a0.clear_errors();
   require(a0.encoder_.spi_error_rate_==0.5f,"clear accepted while a reinit is pending");
   run_one(can);expect_recovery_frame(a0,1,1);  // rejected clear still revoked eligibility
  }else if(name=="pend_arm_rejected"){
   can.set_baud_rate(500000);a0.motor_.armed_state_=Motor::ARMED_STATE_DISARMED;
   safety_critical_arm_motor_pwm(a0.motor_);
   require(a0.motor_.armed_state_==Motor::ARMED_STATE_DISARMED,"pending reinit allowed PWM arm");
  }else if(name=="pend_request_wakes_server"){
   sem_releases=0;can.set_baud_rate(500000);can.reinit_can();can.record_hal_error(HAL_CAN_ERROR_PARAM);
   require(sem_releases==3,"reinit request did not wake the CAN server");
  }else throw std::runtime_error("unknown pend scenario");
#else
  require(false,"patch4 pending-request marking missing");
#endif
```

러너: `cases += ['pend_clear_before_latch', 'pend_arm_rejected', 'pend_request_wakes_server']`

기존 `resume_manual_revokes`/`resume_reinit_revokes`/`resume_invalid_baud` 분기는 수동 요청 직후(서버 실행 전) flags를 본다. D5에서 요청이 즉시 `in_progress`를 세우므로 patch4의 기대값은 `latched|in_progress` = 3이다. 그 분기의 두 `expect_recovery_frame` 줄을 다음으로 바꾼다(patch3 음성 대조는 계속 1을 기대):

```cpp
#if PATCH4
   const uint8_t revoked=3;  // patch4 D5: the request marks in-progress before the server runs
#else
   const uint8_t revoked=1;
#endif
   expect_recovery_frame(a0,name=="resume_invalid_baud"?5:revoked,1);
   expect_recovery_frame(a1,name=="resume_invalid_baud"?5:revoked,1);
```

- [ ] **Step 2: 실패 확인** — 새 3개 FAIL.

- [ ] **Step 3: 구현 — `interface_can.cpp` `request_reinit`** 교체:

```cpp
void ODriveCAN::request_reinit(bool manual) {
    // Mark every axis in progress at enqueue time (patch4 D5): clear and PWM
    // arm are refused from here on. The latch itself stays in the server
    // thread, which cannot preempt the higher-priority axis loop mid-check.
    uint32_t mask = cpu_enter_critical();
    reinit_requested_ = true;
    if (manual) manual_reinit_requested_ = true;
    for (auto axis : axes) {
        axis->can_recovery_in_progress_ = true;
        if (manual) {
            axis->can_recovery_auto_resume_eligible_ = false;
            axis->can_recovery_episode_revoked_ = true;
        }
    }
    cpu_exit_critical(mask);
    osSemaphoreRelease(sem_can);  // never under PRIMASK
}
```

`set_baud_rate()`와 `reinit_can()`에서 Task 2가 넣은 `for (auto axis : axes) axis->can_recovery_auto_resume_eligible_ = false;` 루프(와 그것만 감싸던 critical 구간)를 지운다. `request_reinit(true)`가 같은 취소를 원자적으로 하므로 중복이다. `set_baud_rate`에는 `pending_baud_rate_` 설정 critical 구간만 남긴다.

- [ ] **Step 4: 구현 — `axis.hpp`**

`uint8_t can_recovery_generation_ = 0;` 다음에 추가:

```cpp
    bool can_recovery_episode_revoked_ = false;
```

`latch_can_fault`의 자격 결정부

```cpp
        if (!automatic) {
            can_recovery_auto_resume_eligible_ = false;
        } else if (!can_recovery_latched_) {
```

를 다음으로 바꾼다(나머지 조건식은 그대로):

```cpp
        if (!automatic || can_recovery_episode_revoked_) {
            can_recovery_auto_resume_eligible_ = false;
        } else if (!can_recovery_latched_) {
```

`clear_errors()`의 앞부분(`uint32_t mask = cpu_enter_critical();`부터 `can_recovery_latched_ = false;\n        }`까지)을 다음으로 바꾼다:

```cpp
        uint32_t mask = cpu_enter_critical();
        // Even a rejected early clear revokes automatic-resume provenance for the
        // pending or current episode; the server cannot grant it back (patch4 D5).
        can_recovery_auto_resume_eligible_ = false;
        can_recovery_episode_revoked_ = true;
        // USB and CAN share this gate. A pending reinit, an early clear before
        // IDLE, or a queued arm cannot escape the stop latch.
        if (can_recovery_in_progress_ ||
            (can_recovery_latched_ &&
             (current_state_ != AXIS_STATE_IDLE ||
              (requested_state_ != AXIS_STATE_UNDEFINED && requested_state_ != AXIS_STATE_IDLE)))) {
            cpu_exit_critical(mask);
            return;
        }
        if (can_recovery_latched_) {
            controller_.input_vel_ = 0.0f;
            controller_.input_torque_ = 0.0f;
            controller_.input_pos_ = encoder_.pos_estimate_;
            can_recovery_latched_ = false;
        }
        // An accepted clear ends any latch episode; the next one starts fresh.
        can_recovery_episode_revoked_ = false;
```

- [ ] **Step 5: 구현 — `low_level.cpp`** — `safety_critical_arm_motor_pwm`의 조건을 다음으로:

```cpp
    if (brake_resistor_armed && !motor.axis_->can_recovery_latched_ &&
        !motor.axis_->can_recovery_in_progress_) {
```

- [ ] **Step 6: 통과 확인** — `$WORK/work`: `107 passed, 0 failed`. 기존 `resume_early_clear`, `clear_during_recovery`, `guard_resume_pending_hal`, `resume_manual*`가 계속 PASS인지 특히 확인한다.

- [ ] **Step 7: 음성 대조** — `latch_can_fault`의 `|| can_recovery_episode_revoked_`를 지우면 `pend_clear_before_latch`가 FAIL(flags 5)해야 한다. 원복.

- [ ] **Step 8: 커밋 보류** — Task 6과 함께(Task 6 Step 10).

---

### Task 5: D4 축별 시도 상한

**Files:**
- Modify(작업 트리): `$WORK/work/Firmware/MotorControl/axis.hpp`
- Modify: `firmware/mks_odrive/tests/scenarios.cpp`, `firmware/mks_odrive/tests/run_source_tests.py`

**Interfaces:**
- Consumes: `can_recovery_episode_revoked_`(Task 4).
- Produces: `static constexpr uint8_t kCanRecoveryAttemptLimit = 16;`, `uint8_t can_recovery_attempts_ = 0;`

- [ ] **Step 1: 실패하는 시험 작성** — `scenarios.cpp`, `pend_` 분기 다음:

```cpp
 }else if(name.rfind("k_",0)==0){
#if PATCH4 && HAS_ATTEMPTS
  a0.current_state_=Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;
  if(name=="k_attempt_limit"){
   for(unsigned n=1;n<=16;++n){recover_once(can,h);expect_recovery_frame(a0,5,uint8_t(n));}
   recover_once(can,h);expect_recovery_frame(a0,1,17);
  }else if(name=="k_wrap_revoked"){
   for(unsigned n=0;n<256;++n)recover_once(can,h);expect_recovery_frame(a0,1,0);
   for(unsigned n=0;n<256;++n)recover_once(can,h);expect_recovery_frame(a0,1,0);
  }else if(name=="k_other_axis_clear"){
   a1.current_state_=Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;
   for(unsigned n=0;n<10;++n)recover_once(can,h);
   a1.current_state_=Axis::AXIS_STATE_IDLE;a1.requested_state_=Axis::AXIS_STATE_UNDEFINED;a1.clear_errors();
   require(!a1.can_recovery_latched_,"peer clear rejected");
   for(unsigned n=0;n<7;++n)recover_once(can,h);
   expect_recovery_frame(a0,1,17);
  }else if(name=="k_new_episode"){
   for(unsigned n=0;n<17;++n)recover_once(can,h);
   a0.current_state_=Axis::AXIS_STATE_IDLE;a0.requested_state_=Axis::AXIS_STATE_UNDEFINED;a0.clear_errors();
   require(a0.can_recovery_attempts_==0 && !a0.can_recovery_episode_revoked_,"accepted clear did not end the episode");
   a0.current_state_=Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;recover_once(can,h);expect_recovery_frame(a0,5,18);
  }else throw std::runtime_error("unknown k scenario");
#else
  require(false,"patch4 attempt limit missing");
#endif
```

기존 `resume_generation_wrap` 분기의 `expect_recovery_frame(a0,5,0);`을 `expect_recovery_frame(a0,1,0);`으로 바꾼다(256번 재시도는 K=16을 넘어 자격이 취소된다 — 설계 D4).

러너: `cases += ['k_attempt_limit', 'k_wrap_revoked', 'k_other_axis_clear', 'k_new_episode']`. 아직 없는 필드를 참조해도 컴파일되도록 러너의 `'#define PATCH4 ' ...` 줄 다음에 기능 매크로를 추가한다:

```python
        '#define HAS_ATTEMPTS ' + str(int('can_recovery_attempts_' in axis_header)),
```

- [ ] **Step 2: 실패 확인** — 새 4개와 `resume_generation_wrap` FAIL.

- [ ] **Step 3: 구현 — `axis.hpp`**

`bool can_recovery_episode_revoked_ = false;` 다음에 추가:

```cpp
    static constexpr uint8_t kCanRecoveryAttemptLimit = 16;
    uint8_t can_recovery_attempts_ = 0;  // per-axis latch episode, saturating
```

`latch_can_fault`의 `uint32_t mask = cpu_enter_critical();` 바로 다음에 추가:

```cpp
        if (!can_recovery_latched_) can_recovery_attempts_ = 0;  // new episode
        if (can_recovery_attempts_ < 255) ++can_recovery_attempts_;
```

같은 함수에서 자격 결정 `if/else if` 블록이 끝난 직후(`++can_recovery_generation_;` 앞)에 추가:

```cpp
        // From the 17th attempt of one episode, eligibility never returns until
        // the episode ends, so a live eligible episode cannot wrap the generation.
        if (can_recovery_attempts_ > kCanRecoveryAttemptLimit)
            can_recovery_auto_resume_eligible_ = false;
```

`clear_errors()`의 `can_recovery_episode_revoked_ = false;`(Task 4) 다음에 추가:

```cpp
        can_recovery_attempts_ = 0;
```

- [ ] **Step 4: 러너 필드 추출 확장** — `run_source_tests.py`의 `fields = ...` 줄을 다음으로 바꾼다:

```python
    fields = '\n'.join(re.findall(
        r'^\s*(?:static constexpr (?:uint8_t|uint32_t) kCan\w+ = [^;]+;'
        r'|(?:bool|uint8_t|uint32_t) (?:can_\w+_|control_deadline_count_) = [^;]+;)',
        axis_header, re.M))
```

- [ ] **Step 5: 통과 확인** — `$WORK/work`: `111 passed, 0 failed`.

- [ ] **Step 6: 음성 대조** — `kCanRecoveryAttemptLimit`를 `255`로 바꾸면 `k_attempt_limit`·`k_wrap_revoked` FAIL. 원복.

- [ ] **Step 7: 커밋 보류** — Task 6과 함께.

---

### Task 6: D0 CAN 제어 수신 만료

**Files:**
- Modify(작업 트리): `$WORK/work/Firmware/MotorControl/axis.hpp`, `axis.cpp`, `$WORK/work/Firmware/communication/can_simple.cpp`
- Modify: `firmware/mks_odrive/tests/scenarios.cpp`, `firmware/mks_odrive/tests/run_source_tests.py`

**Interfaces:**
- Consumes(하네스, Task 1): 실제 `run_state_machine_loop`, `meas_hook`, `virtual_tick`, `calibration_ms`.
- Produces(펌웨어, Axis 인라인 메서드): `void can_control_request(AxisState state)`, `void can_control_feed()`, `void can_control_closed_loop_entry()`, `void can_control_idle_entry()`, `bool can_control_check()`; 필드 `can_owner_`, `can_feed_valid_`, `can_entry_recorded_`, `can_last_feed_tick_`, `can_entry_tick_`, `control_deadline_count_`; 상수 `kCanCmdTimeoutMs = 300`, `kCanGraceMs = 500`.

이 Task의 시험·펌웨어 코드는 계획 작성 중 patch3 사본에 D0 변경만 적용해 실행·검증했다(D0 17개 PASS, 아래 음성 대조 결과 포함). 시험 대부분은 **실제 dispatcher·`run_control_loop`·`run_idle_loop`**를 돌린다. `run_axis`는 축 대기마다 tick을 1 ms 올리므로 만료 시각을 정확한 tick으로 단언할 수 있다: 마지막 수락 tick `F`의 만료는 `F+301` 검사에서 일어나고, IDLE에서의 다음 대기 tick `F+302`에 관측된다(유예는 진입 tick `E`에서 `E+502`).

- [ ] **Step 1: 실패하는 시험 작성** — `scenarios.cpp`

러너의 `'#define HAS_ATTEMPTS ' ...` 줄 다음에 추가:

```python
        '#define HAS_D0 ' + str(int('can_control_check' in axis_header)),
```

`int main(` 바로 앞에 추가(파일 범위):

```cpp
#if PATCH4 && HAS_D0
struct AxisRun{uint32_t cl_entries=0,first_exit=0;};
// Runs the real dispatcher until `steps` axis waits; `at(tick)` injects events.
AxisRun run_axis(Axis& a,uint32_t steps,const std::function<void(uint32_t)>& at){
 AxisRun r;bool in_cl=a.current_state_==Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;uint32_t n=0;
 meas_hook=[&](){at(virtual_tick);const bool cl=a.current_state_==Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;
  if(cl && !in_cl)++r.cl_entries;if(!cl && in_cl && !r.first_exit)r.first_exit=virtual_tick;in_cl=cl;
  if(++n>=steps)throw StopLoop{};};
 try{a.run_state_machine_loop();}catch(const StopLoop&){}meas_hook=nullptr;return r;}
void can_state(uint32_t node,uint32_t state){auto m=msg(7,false,4);std::memset(m.buf,0,8);std::memcpy(m.buf,&state,4);m.id=(node<<5)|7;CANSimple::handle_can_message(m);}
void can_vel(uint32_t node){auto m=msg(13,false,8,0.5f,0);m.id=(node<<5)|13;CANSimple::handle_can_message(m);}
#endif
```

`k_` 분기 다음에 추가:

```cpp
 }else if(name.rfind("d0_",0)==0){
#if PATCH4 && HAS_D0
  a0.config_.enable_watchdog=false;a1.config_.enable_watchdog=false;virtual_tick=1000;
  auto watchdog_bit=[&](){return (a0.error_&Axis::ERROR_WATCHDOG_TIMER_EXPIRED)!=0;};
  if(name=="d0_cmd_timeout"){
   // Tick accounting: the feed at hook tick F is checked at the start of each loop
   // iteration; F+300 passes, F+301 expires, and the next wait (in IDLE) is F+302.
   auto r=run_axis(a0,2000,[&](uint32_t t){if(t==1010)can_state(13,8);if(t==1020)can_vel(13);});
   require(r.first_exit==1322,"CAN command deadline not at last feed + 301 ms");
   require(watchdog_bit() && a0.control_deadline_count_==1,"deadline error/count missing");
   require(a0.motor_.armed_state_==Motor::ARMED_STATE_DISARMED,"expiry did not disarm PWM through the real IDLE path");
  }else if(name=="d0_grace"){
   auto r=run_axis(a0,2000,[&](uint32_t t){if(t==1010)can_state(13,8);});
   require(r.first_exit==1512 && watchdog_bit(),"grace not CLOSED_LOOP entry + 501 ms");
  }else if(name=="d0_reentry_can" || name=="d0_reentry_usb"){
   const bool usb=name=="d0_reentry_usb";
   auto r=run_axis(a0,2000,[&](uint32_t t){if(t==1010)can_state(13,8);if(t==1020)can_vel(13);
    if(t==1120||t==1220){if(usb)a0.requested_state_=Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;else can_state(13,8);}});
   require(r.first_exit==1322,"state8 re-entry extended the deadline");
  }else if(name=="d0_usb_only"){
   auto r=run_axis(a0,3000,[&](uint32_t t){if(t==1010)a0.requested_state_=Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;});
   require(r.cl_entries==1 && r.first_exit==0 && !a0.can_owner_ && !watchdog_bit(),"USB-only session supervised by the CAN deadline");
  }else if(name=="d0_usb_then_can_motion"){
   auto r=run_axis(a0,3000,[&](uint32_t t){if(t==1010)a0.requested_state_=Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;if(t==2000)can_vel(13);});
   require(r.first_exit==2302,"CAN motion on a USB-armed axis escaped the deadline");
  }else if(name=="d0_precheck_failure"){
   a0.motor_.config_.direction=0;
   auto r=run_axis(a0,4000,[&](uint32_t t){if(t==1010)can_state(13,8);
    if(t==1100){a0.motor_.config_.direction=1;a0.requested_state_=Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;}});
   require(r.cl_entries==1 && r.first_exit==0 && !watchdog_bit(),"failed CAN arm left ownership that expired a USB session");
  }else if(name=="d0_startup_state2"){
   a0.config_.startup_motor_calibration=true;a0.config_.startup_closed_loop_control=true;calibration_ms=30000;
   auto r=run_axis(a0,40000,[&](uint32_t t){if(t==1010)can_state(13,2);});
   require(r.first_exit==31512 && watchdog_bit(),"CAN startup chain grace not measured from CLOSED_LOOP entry");
  }else if(name=="d0_calibration_only"){
   calibration_ms=1000;
   auto r=run_axis(a0,5000,[&](uint32_t t){if(t==1010)can_state(13,3);});
   require(r.cl_entries==0 && !a0.can_owner_ && !watchdog_bit(),"calibration-only CAN request left a session or expired");
  }else if(name=="d0_can_idle_request"){
   bool owner_after_request=false;
   auto r=run_axis(a0,2000,[&](uint32_t t){if(t==1010)can_state(13,8);if(t==1020)can_vel(13);
    if(t==1050){can_state(13,1);owner_after_request=a0.can_owner_;}});
   require(owner_after_request && r.first_exit==1051 && !a0.can_owner_ && !watchdog_bit(),"CAN IDLE request handling wrong");
  }else if(name=="d0_query_only"){
   auto r=run_axis(a0,2000,[&](uint32_t t){if(t==1010)can_state(13,8);
    if(t>1010){auto q=msg(9,true);CANSimple::handle_can_message(q);auto iq=msg(20,true);CANSimple::handle_can_message(iq);CANSimple::send_heartbeat(&a0);}});
   require(r.first_exit==1512,"queries or heartbeats fed the CAN deadline");
  }else if(name=="d0_tick_wrap"){
   virtual_tick=0xFFFFFF00u-20u;
   auto r=run_axis(a0,1000,[&](uint32_t t){if(t==0xFFFFFF00u-10u)can_state(13,8);if(t==0xFFFFFF00u)can_vel(13);});
   require(r.first_exit==uint32_t(0xFFFFFF00u+302u),"tick wrap changed the deadline");
  }else if(name=="d0_guarded_resume_session"){
   a0.current_state_=Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;recover_once(can,h);
   a0.current_state_=Axis::AXIS_STATE_IDLE;a0.requested_state_=Axis::AXIS_STATE_UNDEFINED;
   auto r=run_axis(a0,2000,[&](uint32_t t){if(t==1010){auto m=guarded_resume(1);CANSimple::handle_can_message(m);}});
   require(r.cl_entries==1 && r.first_exit==1512,"resumed session without input escaped the deadline");
  }else if(name=="d0_idle_concurrent_request"){
   // The window between the dispatcher's request check and the IDLE case is an
   // interrupt race the single-threaded loop cannot hit; drive the hooks directly.
   can_state(13,8);a0.current_state_=Axis::AXIS_STATE_IDLE;a0.can_control_idle_entry();
   require(a0.can_owner_,"IDLE entry dropped a concurrent CAN request");
  }else if(name=="d0_per_axis"){
   can_state(13,8);a0.requested_state_=Axis::AXIS_STATE_UNDEFINED;a0.current_state_=Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;
   a0.can_control_closed_loop_entry();can_vel(13);
   a1.current_state_=Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;
   virtual_tick=1301;require(!a0.watchdog_check() && a1.watchdog_check() && a1.error_==0,"deadline not per axis");
  }else if(name=="d0_idle_feed_ignored"){
   can_vel(13);require(!a0.can_owner_,"IDLE motion created a CAN session");
  }else if(name=="d0_reboot_defaults"){
   Axis fresh;
   require(!fresh.can_owner_ && !fresh.can_feed_valid_ && !fresh.can_entry_recorded_ && fresh.control_deadline_count_==0,"D0 RAM defaults");
   require(fresh.can_recovery_attempts_==0 && !fresh.can_recovery_episode_revoked_,"D4/D5 RAM defaults");
  }else throw std::runtime_error("unknown d0 scenario");
#else
  require(false,"patch4 CAN control deadline missing");
#endif
```

러너 `cases`에 추가:

```python
    cases += ['d0_cmd_timeout', 'd0_grace', 'd0_reentry_can', 'd0_reentry_usb', 'd0_usb_only',
              'd0_usb_then_can_motion', 'd0_precheck_failure', 'd0_startup_state2', 'd0_calibration_only',
              'd0_can_idle_request', 'd0_query_only', 'd0_tick_wrap', 'd0_guarded_resume_session',
              'd0_idle_concurrent_request', 'd0_per_axis', 'd0_idle_feed_ignored', 'd0_reboot_defaults']
```

- [ ] **Step 2: 실패 확인** — 하네스 실행(`$WORK/work`). Expected: 새 17개 FAIL(`patch4 CAN control deadline missing`), 나머지 111 PASS.

- [ ] **Step 3: 구현 — `axis.hpp` 필드·상수** (`uint8_t can_recovery_attempts_ = 0;` 다음)

```cpp
    // patch4 D0: CAN control session (RAM only, outside Config_t/NVM).
    static constexpr uint32_t kCanCmdTimeoutMs = 300;
    static constexpr uint32_t kCanGraceMs = 500;
    bool can_owner_ = false;
    bool can_feed_valid_ = false;
    bool can_entry_recorded_ = false;
    uint32_t can_last_feed_tick_ = 0;
    uint32_t can_entry_tick_ = 0;
    uint32_t control_deadline_count_ = 0;
```

- [ ] **Step 4: 구현 — `axis.hpp` 메서드** (`clear_errors()` 정의 바로 앞)

```cpp
    // Any non-IDLE CAN state request starts (or keeps) a CAN control session.
    // Idempotent: a repeated request never refreshes the timers.
    void can_control_request(AxisState state) {
        if (state == AXIS_STATE_IDLE) return;
        uint32_t mask = cpu_enter_critical();
        if (!can_owner_) {
            can_owner_ = true;
            can_feed_valid_ = false;
            can_entry_recorded_ = false;
        }
        cpu_exit_critical(mask);
    }

    // A validated CAN motion setpoint in CLOSED_LOOP feeds (and if needed
    // claims) the session, also for an axis that USB armed.
    void can_control_feed() {
        uint32_t mask = cpu_enter_critical();
        if (current_state_ == AXIS_STATE_CLOSED_LOOP_CONTROL) {
            const uint32_t now = osKernelSysTick();
            if (!can_owner_) {
                can_owner_ = true;
                can_entry_recorded_ = true;
                can_entry_tick_ = now;
            }
            can_feed_valid_ = true;
            can_last_feed_tick_ = now;
        }
        cpu_exit_critical(mask);
    }

    // The grace period starts at the session's first CLOSED_LOOP entry only.
    void can_control_closed_loop_entry() {
        uint32_t mask = cpu_enter_critical();
        if (can_owner_ && !can_entry_recorded_) {
            can_entry_recorded_ = true;
            can_entry_tick_ = osKernelSysTick();
        }
        cpu_exit_critical(mask);
    }

    // The only session end: actually running IDLE with no request pending.
    void can_control_idle_entry() {
        uint32_t mask = cpu_enter_critical();
        if (requested_state_ == AXIS_STATE_UNDEFINED) {
            can_owner_ = false;
            can_feed_valid_ = false;
            can_entry_recorded_ = false;
        }
        cpu_exit_critical(mask);
    }

    // Deadline relative to the board's last accepted CAN setpoint, independent
    // of enable_watchdog. Unsigned tick differences survive the 49.7 day wrap.
    bool can_control_check() {
        if (current_state_ != AXIS_STATE_CLOSED_LOOP_CONTROL) return true;
        uint32_t mask = cpu_enter_critical();
        const uint32_t now = osKernelSysTick();
        const uint32_t cmd_ticks = kCanCmdTimeoutMs * osKernelSysTickFrequency / 1000u;
        const uint32_t grace_ticks = kCanGraceMs * osKernelSysTickFrequency / 1000u;
        const bool expired = can_owner_ &&
            (can_feed_valid_ ? uint32_t(now - can_last_feed_tick_) > cmd_ticks
                             : can_entry_recorded_ && uint32_t(now - can_entry_tick_) > grace_ticks);
        if (expired) {
            error_ |= ERROR_WATCHDOG_TIMER_EXPIRED;
            ++control_deadline_count_;
        }
        cpu_exit_critical(mask);
        return !expired;
    }
```

- [ ] **Step 5: 구현 — `axis.cpp`**

`Axis::watchdog_check()`의 첫 주석 앞에 추가:

```cpp
    // patch4 D0: the CAN control deadline is independent of enable_watchdog.
    if (!can_control_check()) return false;
```

상태머신의 CLOSED_LOOP case를:

```cpp
            case AXIS_STATE_CLOSED_LOOP_CONTROL: {
                if (!motor_.is_calibrated_ || motor_.config_.direction==0)
                    goto invalid_state_label;
                if (!encoder_.is_ready_)
                    goto invalid_state_label;
                watchdog_feed();
                can_control_closed_loop_entry();
                status = run_closed_loop_control_loop();
            } break;
```

IDLE case를:

```cpp
            case AXIS_STATE_IDLE: {
                can_control_idle_entry();
                run_idle_loop();
                status = motor_.arm(); // done with idling - try to arm the motor
            } break;
```

- [ ] **Step 6: 구현 — `can_simple.cpp`**

`handle_can_message`의 `if (control_input) axis->watchdog_feed();`를 다음으로:

```cpp
        if (control_input) {
            axis->watchdog_feed();
            axis->can_control_feed();
        }
```

`set_axis_requested_state_callback`의 guarded RESUME 커밋부 `axis->requested_state_ = Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;` 다음 줄에 추가:

```cpp
            axis->can_control_request(Axis::AXIS_STATE_CLOSED_LOOP_CONTROL);
```

같은 함수 마지막 줄 `axis->requested_state_ = static_cast<Axis::AxisState>(can_getSignal<int32_t>(msg, 0, 32, true));`을 다음으로:

```cpp
    const auto state = static_cast<Axis::AxisState>(can_getSignal<int32_t>(msg, 0, 32, true));
    uint32_t mask = cpu_enter_critical();
    axis->requested_state_ = state;
    axis->can_control_request(state);  // same critical section as the request
    cpu_exit_critical(mask);
```

- [ ] **Step 7: 러너 메서드 추출** — `axis_methods` 조립부(`if 'void latch_can_fault(' in axis_header:` 앞)에 추가:

```python
    for signature in ('void can_control_request(', 'void can_control_feed()',
                      'void can_control_closed_loop_entry()', 'void can_control_idle_entry()',
                      'bool can_control_check()'):
        if signature in axis_header:
            axis_methods += '\n' + function(axis_header, signature)
```

- [ ] **Step 8: 통과 확인** — `$WORK/work`: `128 passed, 0 failed`. 기존 `watchdog_expiry`·`idle_watchdog`·`valid_control`·`harness_state_machine`도 PASS여야 한다.

- [ ] **Step 9: 음성 대조(계획 작성 중 실측한 결과)** —
  - (a) `run_control_loop`의 `bool watchdog_ok = watchdog_check();`를 `bool watchdog_ok = (watchdog_check(), true);`로 바꾸면 실제 루프 시험 9개(`d0_cmd_timeout`·`d0_grace`·`d0_reentry_can`·`d0_reentry_usb`·`d0_usb_then_can_motion`·`d0_startup_state2`·`d0_query_only`·`d0_tick_wrap`·`d0_guarded_resume_session`)가 FAIL해야 한다.
  - (b) `can_control_check`의 `> cmd_ticks`를 `>= cmd_ticks`로 바꾸면 5개(`d0_cmd_timeout`·`d0_reentry_can`·`d0_reentry_usb`·`d0_usb_then_can_motion`·`d0_tick_wrap`)가 FAIL해야 한다.
  - (c) `can_control_request`의 `if (!can_owner_)` 조건을 지우면 `d0_reentry_can`이 FAIL해야 한다.
  - (d) `can_control_idle_entry`의 `requested_state_ == AXIS_STATE_UNDEFINED` 조건을 지우면 `d0_idle_concurrent_request`가 FAIL해야 한다.
  각각 확인 후 원복한다.

- [ ] **Step 10: Codex 리뷰(교차검증)** — Task 4~6 diff를 Task 3 Step 8과 같은 방식으로 보낸다. 질문에 반드시 포함: `d0_idle_concurrent_request`·`d0_per_axis`만 메서드 수준이라는 점(전자는 dispatcher의 요청 검사와 IDLE case 사이 인터럽트 경합이라 단일 스레드 루프로 재현할 수 없음), `can_control_feed`가 CAN 서버 스레드에서 불린다는 전제, `latch_can_fault`가 서버 스레드에서만 불린다는 전제. 새 P1/P2가 없을 때까지 반복.

- [ ] **Step 11: 패치 재생성·커밋(사용자 확인 후)**

```bash
$PY firmware/mks_odrive/regen_patch.py --original $WORK/original --work $WORK/work
MKS_VENDOR_ZIP=$ZIP $PY firmware/mks_odrive/tests/test_regen_patch.py
git add firmware/mks_odrive/reliability.patch firmware/mks_odrive/tests/scenarios.cpp firmware/mks_odrive/tests/run_source_tests.py
git commit -m "fix(firmware): CAN 제어 수신 만료·복구 시도 상한·재초기화 대기 중 clear 차단을 더한다"
```

---

### Task 7: patch4 식별자·Fibre 노출·문서·빌드

**Files:**
- Modify(작업 트리): `interface_can.hpp`(식별자), `odrive-interface.yaml`, `tools/odrive/mks_reliability_version.py`
- Modify: `firmware/mks_odrive/prepare_source.py`, `firmware/mks_odrive/tests/test_prepare.py`, `firmware/mks_odrive/tests/scenarios.cpp`, `firmware/mks_odrive/README.md`
- Regenerate: `firmware/mks_odrive/reliability.patch`

- [ ] **Step 1: 실패하는 시험으로 기대값 변경**

`scenarios.cpp`의 `resume_patch_identity` 분기:

```cpp
   require(can.reliability_patch_==4,"patch4 identity missing");
```

`test_prepare.py`의 `self.assertEqual(manifest['reliability_patch'], 3)` → `4`. 같은 시험에 추가:

```python
            yaml = (dest / 'Firmware/odrive-interface.yaml').read_text()
            for name in ('lec_error_count', 'rx_overrun_count', 'bus_off_count',
                         'error_passive_count', 'error_warning_count', 'control_deadline_count'):
                self.assertIn(f'{name}: readonly uint32', yaml)
            self.assertIn('can.reliability_patch=4', (dest / 'tools/odrive/mks_reliability_version.py').read_text())
```

- [ ] **Step 2: 실패 확인** — 하네스 `resume_patch_identity` FAIL, `test_prepare.py` FAIL.

- [ ] **Step 3: 구현**

`interface_can.hpp`: `const uint32_t reliability_patch_ = 3;` → `4`.

`odrive-interface.yaml`의 `ODrive.Can` attributes에서 `reinit_requested: readonly bool` 다음에 추가:

```yaml
      lec_error_count: readonly uint32
      rx_overrun_count: readonly uint32
      bus_off_count: readonly uint32
      error_passive_count: readonly uint32
      error_warning_count: readonly uint32
```

`ODrive.Axis` attributes에서 `can_recovery_generation: readonly uint8` 다음에 추가:

```yaml
      control_deadline_count: readonly uint32
```

`tools/odrive/mks_reliability_version.py`: 독스트링과 마지막 print의 `patch 3`/`can.reliability_patch=3`을 `patch 4`/`can.reliability_patch=4`로.

`prepare_source.py`: 상단 상수에 `RELIABILITY_PATCH = 4`를 추가하고 manifest의 `'reliability_patch': 3 if patch else 0`을 `RELIABILITY_PATCH if patch else 0`으로.

- [ ] **Step 4: README 갱신** — `firmware/mks_odrive/README.md`

  - 제목과 첫 문단: `patch 3` → `patch 4`, 설계 문서 링크 `[patch4 설계](../../docs/specs/2026-10-15-mks-can-patch4.md)` 추가.
  - "소스 준비와 실행 시험"의 기대값을 Step 6에서 실제로 나온 숫자로 바꾼다(원본 RED, patch4 GREEN, patch3 음성 대조 결과 3줄). 숫자는 실행 결과를 그대로 옮긴다.
  - "런타임 계약"에 다음 항목을 추가하고, patch3의 "HAL 오류 = 재초기화" 서술을 D1 분류로 고친다: 정보성 오류 계수·비정지(D1), 자기 node 수신 필터와 송신 ID 범위 검사(D2), 성공 후 즉시 RX 서비스(D3), 축별 시도 상한 16(D4), 요청 시점 in-progress 표시·clear/arm 거절(D5), CAN 제어 수신 만료 300/500 ms와 세션 규칙·`startup_*`=false 필수 전제(D0). 각 항목은 설계 문서의 해당 절과 같은 수치·조건을 쓴다.
  - "읽기 전용 Fibre 진단" 표에 새 계수 6개를 추가한다.

- [ ] **Step 5: Cortex-M4 빌드**

```bash
$PY firmware/mks_odrive/regen_patch.py --original $WORK/original --work $WORK/work
$PY firmware/mks_odrive/build_firmware.py --zip $ZIP --output $WORK/build-p4
$PY -c "import json;m=json.load(open('$WORK/build-p4/mks-build-manifest.json'));print(m['source']['reliability_patch']);[print(k,v['size_bytes'],v['sha256']) for k,v in m['artifacts'].items()]"
```
Expected: 빌드 성공(스크립트가 마지막에 `<name>: <bytes> bytes sha256=...`를 출력), 첫 줄 `4`, ELF·BIN 두 줄. BIN 크기가 STM32F405 application 영역(768 KiB = 786432 B) 이하인지 확인한다.

- [ ] **Step 6: 전체 시험**

```bash
MKS_VENDOR_ZIP=$ZIP $PY firmware/mks_odrive/tests/test_prepare.py --zip $ZIP
MKS_VENDOR_ZIP=$ZIP $PY firmware/mks_odrive/tests/test_regen_patch.py
$PY firmware/mks_odrive/tests/test_build_firmware.py
$PY firmware/mks_odrive/tests/run_source_tests.py $WORK/work | tail -1
$PY firmware/mks_odrive/tests/run_source_tests.py $WORK/patch3 | tail -1
$PY firmware/mks_odrive/tests/run_source_tests.py $WORK/original | tail -1
```
Expected: 앞의 세 개 OK, 개발본 `128 passed, 0 failed`, patch3·원본은 새 시험이 FAIL하는 숫자. 이 세 줄을 README Step 4 기대값에 그대로 적는다.

- [ ] **Step 7: 커밋(사용자 확인 후)**

```bash
git add firmware/mks_odrive/reliability.patch firmware/mks_odrive/prepare_source.py \
  firmware/mks_odrive/tests/test_prepare.py firmware/mks_odrive/tests/scenarios.cpp firmware/mks_odrive/README.md
git commit -m "feat(firmware): MKS CAN patch4 식별자와 진단 계수를 노출하고 문서를 갱신한다"
```

---

### Task 8: 브랜치 전체 교차검증

- [ ] **Step 1: Codex 적대적 리뷰** — 브랜치 전체를 대상으로:

```bash
node <codex-plugin>/scripts/codex-companion.mjs adversarial-review --base main "MKS CAN patch4 against docs/specs/2026-10-15-mks-can-patch4.md: D0-D5 contracts, harness boundary honesty, NVM/wire compatibility" > /tmp/codex-branch.md 2>&1
```
출력을 자르지 말고 파일로 읽는다. P1/P2마다 소스로 근거를 확인하고, 수정은 해당 Task의 TDD 순서(실패 시험 → 구현 → 통과)로 한다. 같은 스레드로 재검토를 요청해 "새 P1/P2 없음"까지 반복한다.

- [ ] **Step 2: 독립 검토** — 설계 문서 6절과 Codex 결론을 주지 않은 새 스레드(`task`, `--resume-last` 없이)에 브랜치 diff와 설계 문서 1~5절만 주고 "적히지 않은 전제와 구현이 설계와 어긋나는 곳"을 묻는다. 지적을 반영하고 같은 스레드로 수렴까지 반복한다.

- [ ] **Step 3: 결과 기록** — `docs/plans/2026-10-15-mks-can-patch4.md` 끝의 "구현 검증 로그"에 리뷰 지적·처리·최종 시험 숫자를 적고 커밋한다(사용자 확인 후).

---

### Task 9: 실물 인수 도구 — NVM 인수 점검과 CAN 송신 순서 측정

**Files:**
- Create: `tools/mks_nvm_acceptance.py`, `tools/tests/test_mks_nvm_acceptance.py`
- Create: `tools/can_tx_order_probe.py`, `tools/tests/test_can_tx_order_probe.py`

**Interfaces:**
- `mks_nvm_acceptance.check(configs: dict[str, dict], expected: set[str]) -> list[str]` — 보드 serial → 설정 JSON(기존 증거 형식: `axis0/axis1.config.*`, `can.config.*`). `expected`는 반드시 검사해야 하는 serial 전체 목록이며, 빠지거나 남는 보드도 위반이다. 위반 문자열 목록(빈 목록 = 통과).
- `can_tx_order_probe.analyze(sent, echoed) -> dict` — `sent`는 송신 순서의 `(seq, 송신 직전 time.time())`, `echoed`는 도착 순서의 `(seq, 커널 echo 타임스탬프)`. `accepted`는 **전량 관측 + 송신 순서와 완전 일치**일 때만 true.

- [ ] **Step 1: 실패하는 시험** — `tools/tests/test_mks_nvm_acceptance.py`

```python
import copy
import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
import mks_nvm_acceptance as nvm  # noqa: E402

EVIDENCE = ROOT / 'docs/reports/2026-09-10-can-auto-resume-evidence/board-336A33523235-patch3/336A33523235-before-configuration.json'


def board(n0, n1):
    cfg = copy.deepcopy(json.loads(EVIDENCE.read_text()))
    cfg['axis0']['config']['can_node_id'], cfg['axis1']['config']['can_node_id'] = n0, n1
    return cfg


class NvmAcceptanceTest(unittest.TestCase):
    def boards(self):
        return {'A': board(11, 12), 'B': board(13, 14), 'C': board(15, 16)}

    def test_real_evidence_shape_passes(self):
        self.assertEqual(nvm.check(self.boards(), {'A', 'B', 'C'}), [])

    def test_missing_or_extra_board_fails(self):
        boards = self.boards()
        del boards['A']
        self.assertIn('missing board A', nvm.check(boards, {'A', 'B', 'C'}))
        self.assertIn('unexpected board B', nvm.check(self.boards(), {'A', 'C'}))

    def test_startup_flag_fails(self):
        boards = self.boards()
        boards['B']['axis1']['config']['startup_closed_loop_control'] = True
        self.assertIn('B axis1 startup_closed_loop_control=True', nvm.check(boards, {'A', 'B', 'C'}))

    def test_cross_board_duplicate_fails(self):
        boards = self.boards()
        boards['C']['axis0']['config']['can_node_id'] = 13
        self.assertTrue(any('duplicate node 13' in v for v in nvm.check(boards, {'A', 'B', 'C'})))

    def test_baud_protocol_range_fail(self):
        boards = self.boards()
        boards['A']['can']['config']['baud_rate'] = 250000
        boards['A']['can']['config']['protocol'] = 1
        boards['B']['axis0']['config']['can_node_id'] = 77
        found = nvm.check(boards, {'A', 'B', 'C'})
        self.assertIn('A can baud_rate=250000', found)
        self.assertIn('A can protocol=1', found)
        self.assertIn('B axis0 node 77 out of STD range', found)

    def test_probe_node_and_silent_axis_fail(self):
        boards = self.boards()
        boards['B']['axis0']['config']['can_node_id'] = 63
        boards['B']['axis0']['config']['can_heartbeat_rate_ms'] = 0
        found = nvm.check(boards, {'A', 'B', 'C'})
        self.assertIn('B axis0 node 63 is reserved for the CAN probe', found)
        self.assertIn('B axis0 can_heartbeat_rate_ms=0', found)


if __name__ == '__main__':
    unittest.main(verbosity=2)
```

`tools/tests/test_can_tx_order_probe.py`:

```python
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import can_tx_order_probe as probe  # noqa: E402


class AnalyzeTest(unittest.TestCase):
    def test_complete_in_order_is_accepted(self):
        result = probe.analyze([(0, 0.0), (1, 0.001)], [(0, 0.0005), (1, 0.0030)])
        self.assertTrue(result['accepted'])
        self.assertEqual(result['missing'], [])
        self.assertAlmostEqual(result['max_ms'], 2.0, places=6)

    def test_reordered_or_missing_is_rejected(self):
        result = probe.analyze([(0, 0.0), (1, 0.0), (2, 0.0)], [(1, 0.1), (0, 0.2)])
        self.assertFalse(result['accepted'])
        self.assertEqual(result['missing'], [2])

    def test_nothing_observed_is_rejected(self):
        result = probe.analyze([(0, 0.0), (1, 0.0)], [])
        self.assertFalse(result['accepted'])
        self.assertEqual(result['missing'], [0, 1])

    def test_duplicate_echo_is_rejected(self):
        self.assertFalse(probe.analyze([(0, 0.0)], [(0, 0.1), (0, 0.2)])['accepted'])

    def test_inventory_report_is_required(self):
        probe.require_inventory({'pass': True, 'expected': ['A', 'B', 'C'], 'violations': []})
        for bad in ({'pass': False, 'expected': ['A', 'B', 'C'], 'violations': ['x']},
                    {'pass': True, 'expected': ['A'], 'violations': []}):
            with self.assertRaises(SystemExit):
                probe.require_inventory(bad)

    def test_probe_identifier_is_node_63_only(self):
        self.assertEqual(probe.probe_id(7), (0x3F << 5) | 7)
        with self.assertRaises(ValueError):
            probe.probe_id(32)


if __name__ == '__main__':
    unittest.main(verbosity=2)
```

- [ ] **Step 2: 실패 확인** — 두 파일 실행 → `ModuleNotFoundError`.

- [ ] **Step 3: 구현** — `tools/mks_nvm_acceptance.py`

```python
#!/usr/bin/env python3
"""patch4 NVM acceptance (spec 5절 0번): read-only check of saved board configs."""
import argparse
import json
from pathlib import Path

STARTUP = ('startup_motor_calibration', 'startup_encoder_index_search',
           'startup_encoder_offset_calibration', 'startup_closed_loop_control',
           'startup_sensorless_control', 'startup_homing')


def check(configs, expected):
    violations = [f'missing board {s}' for s in sorted(set(expected) - set(configs))]
    violations += [f'unexpected board {s}' for s in sorted(set(configs) - set(expected))]
    owners = {}
    for serial, cfg in sorted(configs.items()):
        can = cfg['can']['config']
        if can['baud_rate'] != 500000:
            violations.append(f"{serial} can baud_rate={can['baud_rate']}")
        if can['protocol'] != 0:
            violations.append(f"{serial} can protocol={can['protocol']}")
        for axis in ('axis0', 'axis1'):
            ac = cfg[axis]['config']
            for flag in STARTUP:
                if ac.get(flag) is not False:
                    violations.append(f'{serial} {axis} {flag}={ac.get(flag)}')
            node, ext = ac['can_node_id'], ac['can_node_id_extended']
            limit = 0xFFFFFF if ext else 0x3F
            if not 0 <= node <= limit:
                violations.append(f"{serial} {axis} node {node} out of {'EXT' if ext else 'STD'} range")
            if not ext and node == 0x3F:
                violations.append(f'{serial} {axis} node 63 is reserved for the CAN probe')
            if ac['can_heartbeat_rate_ms'] <= 0:
                violations.append(f"{serial} {axis} can_heartbeat_rate_ms={ac['can_heartbeat_rate_ms']}")
            key = (node, ext)
            if key in owners:
                violations.append(f'duplicate node {node} ({owners[key]}, {serial} {axis})')
            owners[key] = f'{serial} {axis}'
    return violations


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--expect', action='append', required=True, help='serial that must be present (repeat)')
    ap.add_argument('--json-out', help='write {pass, expected, violations} for can_tx_order_probe')
    ap.add_argument('configs', nargs='+', help='SERIAL=path/to/config.json')
    args = ap.parse_args()
    configs = {}
    for item in args.configs:
        serial, path = item.split('=', 1)
        if serial in configs:
            raise SystemExit(f'duplicate config argument for {serial}')
        configs[serial] = json.loads(Path(path).read_text())
    found = check(configs, set(args.expect))
    if args.json_out:
        Path(args.json_out).write_text(json.dumps(
            {'pass': not found, 'expected': sorted(set(args.expect)), 'violations': found}, indent=2) + '\n')
    for line in found:
        print('FAIL', line)
    print('PASS' if not found else f'{len(found)} violation(s)')
    raise SystemExit(1 if found else 0)


if __name__ == '__main__':
    main()
```

`tools/can_tx_order_probe.py`:

```python
#!/usr/bin/env python3
"""Measure same-ID TX order and queue latency on SocketCAN from own-message echoes.

Sends only on STD node 63 (0x3F). Requires a passing mks_nvm_acceptance report for
all drive boards (which reserves node 63 and forbids silent axes); AK servos use EXT IDs
(motor_control/steering/ak_control.py). It also listens first and aborts on node 63 traffic.
Latency = kernel echo timestamp - wall time taken just before send(); both CLOCK_REALTIME.
"""
import argparse
import json
import struct
import subprocess
import threading
import time

PROBE_NODE = 0x3F
MARKER = 0xA5A5


def probe_id(cmd):
    if not 0 <= cmd < 32:
        raise ValueError('command must be 0..31')
    return (PROBE_NODE << 5) | cmd


def analyze(sent, echoed):
    sent_seq = [seq for seq, _ in sent]
    echo_seq = [seq for seq, _ in echoed]
    sent_time = dict(sent)
    latencies = sorted((t - sent_time[seq]) * 1000.0 for seq, t in echoed if seq in sent_time)

    def pct(q):
        return latencies[min(len(latencies) - 1, int(q * (len(latencies) - 1) + 0.5))] if latencies else None
    return {'accepted': bool(sent_seq) and echo_seq == sent_seq,
            'missing': sorted(set(sent_seq) - set(echo_seq)),
            'echoed': len(echo_seq), 'sent': len(sent_seq),
            'p50_ms': pct(0.5), 'p99_ms': pct(0.99), 'max_ms': latencies[-1] if latencies else None}


def require_inventory(report, minimum_boards=3):
    if report.get('pass') is not True or report.get('violations'):
        raise SystemExit('NVM acceptance did not pass; refusing to transmit')
    if len(report.get('expected', [])) < minimum_boards:
        raise SystemExit('NVM acceptance did not cover every drive board; refusing to transmit')


def echo_enabled(channel):
    out = subprocess.run(['ip', '-details', 'link', 'show', channel],
                         capture_output=True, text=True, check=True).stdout
    flags = out.split('<', 1)[1].split('>', 1)[0].split(',')
    return 'ECHO' in flags


def run(channel, cmd, count, gap_s, listen_s, timeout_s, report):
    import can  # python-can, present in the Jetson container
    require_inventory(report)
    can_id = probe_id(cmd)
    if not echo_enabled(channel):
        raise SystemExit(f'{channel} lacks IFF_ECHO: echoes would not mean TX completion')
    bus = can.Bus(interface='socketcan', channel=channel, receive_own_messages=True)
    try:
        end = time.time() + listen_s
        while time.time() < end:
            msg = bus.recv(timeout=0.05)
            if msg is not None and not msg.is_extended_id and msg.arbitration_id >> 5 == PROBE_NODE:
                raise SystemExit(f'node 63 already in use on the bus (0x{msg.arbitration_id:X}); refusing')
        sent, echoed = [], []
        deadline = time.time() + timeout_s

        def receive():
            while len(echoed) < count and time.time() < deadline:
                msg = bus.recv(timeout=0.05)
                if (msg is not None and msg.arbitration_id == can_id and len(msg.data) == 8
                        and struct.unpack('<H', bytes(msg.data[4:6]))[0] == MARKER):
                    echoed.append((struct.unpack('<I', bytes(msg.data[:4]))[0], msg.timestamp))
        reader = threading.Thread(target=receive)
        reader.start()
        for seq in range(count):
            sent.append((seq, time.time()))  # before send(): includes any queueing inside send
            bus.send(can.Message(arbitration_id=can_id, is_extended_id=False,
                                 data=struct.pack('<IHxx', seq, MARKER)), timeout=1.0)
            if gap_s:
                time.sleep(gap_s)
        reader.join()
    finally:
        bus.shutdown()
    return analyze(sent, echoed)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--channel', default='can0')
    ap.add_argument('--cmd', type=int, default=7, help='command bits on node 63 (0..31)')
    ap.add_argument('--count', type=int, default=1000)
    ap.add_argument('--gap-ms', type=float, default=0.0)
    ap.add_argument('--listen-s', type=float, default=2.0)
    ap.add_argument('--timeout-s', type=float, default=10.0)
    ap.add_argument('--nvm-report', required=True, help='mks_nvm_acceptance --json-out result')
    args = ap.parse_args()
    with open(args.nvm_report) as stream:
        report = json.load(stream)
    print(json.dumps(run(args.channel, args.cmd, args.count, args.gap_ms / 1000.0,
                         args.listen_s, args.timeout_s, report)))


if __name__ == '__main__':
    main()
```

- [ ] **Step 4: 통과 확인** — 두 시험 파일 실행: 전부 OK. 실제 증거 파일로 한 번 실행해 PASS 출력 확인:

```bash
E=docs/reports/2026-09-10-can-auto-resume-evidence/board-336A33523235-patch3/336A33523235-before-configuration.json
$PY tools/mks_nvm_acceptance.py --expect 336A33523235 336A33523235=$E
```

- [ ] **Step 5: Codex 리뷰·커밋(사용자 확인 후)**

```bash
git add tools/mks_nvm_acceptance.py tools/tests/test_mks_nvm_acceptance.py tools/can_tx_order_probe.py tools/tests/test_can_tx_order_probe.py
git commit -m "feat(tools): patch4 실물 인수용 NVM 점검과 CAN 송신 순서 측정 도구를 더한다"
```

---

### Task 10: 실물 인수 (사용자 승인 게이트)

이 Task의 모든 단계는 **사용자가 그 단계를 명시적으로 승인한 뒤에만** 실행한다. 각 단계 결과는 `docs/reports/2026-10-XX-mks-can-patch4.md`와 증거 폴더에 남긴다. Jetson 접속은 `SSHPASS="$JETSON_SSH_PASS" sshpass -e ssh jetson`, 모터·CAN 파이썬은 컨테이너 안에서 실행한다(`.claude/CLAUDE.md` §8). 시작 전 좀비 teleop/제어 프로세스를 확인·정리한다.

- [ ] **Step 0: NVM 인수(읽기 전용)** — 세 보드를 USB로 연결해 기존 증거 스크립트 방식(`docs/reports/2026-09-10-can-auto-resume-evidence/board-*/backup_current.py`의 설정 덤프)으로 설정 JSON을 저장하고 `tools/mks_nvm_acceptance.py --expect <13/14 serial 336A33523235> --expect <15/16 serial 337733643235> --expect <node11/12 serial> ...`로 세 보드를 한꺼번에 점검한다(node11/12 serial은 이 단계에서 USB로 처음 식별해 기록한다). `--json-out nvm-acceptance.json`으로 결과를 저장한다(Step 6의 송신 도구가 이 PASS 보고서를 요구한다). node11/12 보드는 읽기만 한다. 위반이 있으면 해당 보드는 중단하고 보고한다.
- [ ] **Step 1: 13/14 보드 플래시** — 정확한 serial `336A33523235`, 1 MiB 2회 백업 대조, `tools/mks_dfu.py`의 application 전용 guarded 경로로 `$WORK/build-p4/ODriveFirmware.bin` 기록, NVM·설정·캘리 읽기 대조, `can.reliability_patch == 4` 확인, 부팅 후 Step 0 재점검.
- [ ] **Step 2: 실트래픽 10분(바퀴 들고)** — 트래픽 발생원은 운용 경로(ChassisManager, 무장 중 50 Hz setpoint)다. motor_gui `odrive_can` 트랙이나 `can_drive_test.py`는 setpoint를 300 ms 넘게 끊어 D0로 만료되므로 이 시험의 발생원으로 쓰지 않는다. 10모터 운용 트래픽에서 `recovery_count` 증가 0, 래치 0, 정보성 계수 기록.
- [ ] **Step 3: Jetson 가지만 분리, 다른 노드 ACK 유지, 0속도 CLOSED_LOOP** — D0 만료 정지와 시간, `control_deadline_count` 증가를 기록한다.
- [ ] **Step 4: 보드 단독 고립과 수동 재초기화** — 기대 결과를 명시한다. 수동 재초기화(`reinit_can()`·`set_baud_rate()`)는 자격을 취소하므로 **guarded RESUME은 거절돼야 한다**(heartbeat flags: 래치 1·자격 0). 이어 IDLE 확인 → 수동 clear → 0속도 → 명시 arm의 수동 재시작 경로가 동작함을 기록한다. 자동 HAL 복구 뒤 guarded RESUME의 **수락** 경로는 patch4에서 API 실패·HAL 상태 이상으로만 생겨 안전하게 유발할 수 없으므로, 소스 하네스(`guard_*`, `d0_guarded_resume_session`)가 근거이며 실물에서는 미검증으로 보고한다. 같은 단계에서 **요청→래치 지연(D5)**을 잰다: USB로 `reinit_can()` 호출 직전 시각부터 `axisN.can_recovery_latched == True`, `can.reinitializing == False`를 USB로 최대한 빠르게 폴링해 각 시각을 기록한다(USB 폴링 주기가 분해능 하한이며 그 값도 함께 적는다).
- [ ] **Step 5: USB 스키드 경로 무영향** — USB로만 arm·구동해 만료가 나지 않음을 기록한다.
- [ ] **Step 6: 송신 순서·큐 지연** — 시작 전 `ip -details link show can0`에 `ECHO` 플래그가 있는지 확인한다(없으면 echo가 송신 완료를 뜻하지 않으므로 이 측정은 무효). Jetson 컨테이너에서 `python3 tools/can_tx_order_probe.py --nvm-report nvm-acceptance.json --count 1000`을 (a) 평상 트래픽, (b) `--gap-ms 0` 포화, (c) Jetson 가지 일시 분리로 버스 오류·bus-off·restart를 일으킨 조건에서 실행한다. 각 실행 전후 `tc -s qdisc show dev can0`와 `ip -s -details link show can0`(오류·bus-off·restart 계수)를 기록해 (c)에서 실제로 오류 상태를 거쳤음을 증명한다. 도구는 node 63(0x3F)만 쓰며 먼저 2초 수신해 그 node가 버스에 있으면 중단한다. **세 조건 모두 `accepted == true`(전량 관측 + 송신 순서 완전 일치)일 때만** D4 게이트 통과다. 하나라도 false이거나 (c)에서 오류 상태 증거가 없으면 자동 재개 의존 운용을 보류한다.
- [ ] **Step 7: arm→첫 setpoint 시간** — 6축 arm에서 첫 축 arm부터 첫 setpoint까지를 raw CAN 캡처로 측정해 500 ms 대비 여유를 기록한다.
- [ ] **Step 8: 15/16 보드 확대** — Step 1~5를 serial `337733643235`에 반복한다.
- [ ] **Step 9: 보고서와 지침 갱신** — 결과 보고서를 쓰고, `.claude/CLAUDE.md`와 `AGENTS.md`의 §2·§6 MKS 항목을 patch4 상태로 함께 갱신한다(커밋은 사용자 확인 후).

---

## 계획 자체 검토 기록

(아래는 계획 작성 뒤 수행한 검토의 지적과 처리를 적는 곳이다. 구현 중 발견한 사항은 "구현 검증 로그"에 따로 적는다.)

### ① 원자료 대조 (Claude, 2026-10-15)

- 수정 대상 9개 TX 생성 지점을 patch3 소스에서 다시 셌다(`grep -c "can_node_id << NUM_CMD_ID_BITS"` = 9, 함수·MSG 대응은 Task 3 표). `get_vbus_voltage_callback`도 `if (msg.rtr)` 안에 있어 시험의 RTR 호출로 도달한다.
- IRQ 핸들러가 부르는 콜백 13개가 원본·patch3 `interface_can.cpp`에 모두 정의돼 있음을 확인했다(실제 핸들러를 세 소스 모두에 링크 가능).
- `osKernelSysTick`은 `odrive_main.h:26`의 `cmsis_os.h`가 `axis.hpp`(같은 파일 212행 include)보다 먼저 들어와 펌웨어에서 쓸 수 있다.
- HAL `ConfigFilter`는 bank를 먼저 비활성화하고 `FilterActivation == ENABLE`일 때만 다시 켠다(`stm32f4xx_hal_can.c:540` 부근) → 범위 밖 node에 `DISABLE`을 주면 그 bank는 아무것도 받지 않는다.
- `build_firmware.py` 매니페스트 키(`source.reliability_patch`, `artifacts.<name>.size_bytes/sha256`)를 소스에서 확인해 Task 7 명령에 반영했다.
- **Task 1 시제품 실행:** Task 1의 하네스 변경을 저장소 밖 사본에 그대로 적용해 실행했다. patch3 `87 passed, 0 failed`, 원본 `5 passed, 82 failed` — 계획의 기대값과 일치. 실제 헤더 상수 주입이 기존 86개 결과를 바꾸지 않음을 확인했다.
- 고친 것: 불완전 타입 전방 선언 제거, `resume_manual_at_dispatch`의 critical 구간 횟수 의존 트리거를 래치 기준으로 교체(Task 2), `apply_can_config`의 `Init` 실패도 재초기화 요청 없이 기록만(Task 3), 중복 자격 취소 루프 정리(Task 4), guarded RESUME 시나리오 사전 상태, 빌드 매니페스트 읽기, 송신 순서 도구의 `is_rx` 의존 제거.

### ② Codex 적대적 검토 · ③ 독립 검토 (2026-10-15, 서로 결론 비공개 상태로 병렬 실행)

두 검토 모두 "미수렴". 지적과 처리:

| 지적 | 출처 | 처리 |
|---|---|---|
| P1 Task 3 시험이 `CANSimple` private 콜백을 호출해 컴파일 실패 | ②③ | 러너가 하네스 안에서만 `can_simple.hpp`의 `private:`를 `public:`으로 바꾸게 함(Task 3 Step 1) |
| P2 D5로 기존 `resume_manual_revokes`·`resume_reinit_revokes` flags가 1→3 | ②③ | patch4 기대값 3, patch3 음성 대조 1을 `#if PATCH4`로 명시(Task 4 Step 1) |
| P2 D0 시험이 상태를 직접 대입해 실제 루프 탈출·PWM 차단을 검증하지 않음(`watchdog_ok` 강제 true 변이가 통과) | ②③ | 실제 `run_state_machine_loop`·`run_control_loop`·`run_closed_loop_control_loop`·`run_idle_loop`를 하네스에 추출(Task 1)하고 D0 시험 13개를 실제 dispatcher로 재작성(Task 6). 문자열 구조 시험은 제거. **patch3 사본에 D0만 적용해 실측: D0 17/17 PASS, `watchdog_ok` 강제 true 변이 9개 FAIL, `>=` 경계 변이 5개 FAIL** |
| P2 송신 순서 게이트가 관측 0개도 통과 | ②③ | `accepted` = 전량 관측 + 송신 순서 완전 일치. 빈 관측·중복 echo 시험 추가(Task 9). Task 10 게이트를 `accepted`로 |
| P2 큐 지연에 사용자 공간 수신 대기가 섞임 | ②③ | 수신 스레드를 송신과 동시에 돌리고, 송신 직전 `time.time()`과 커널 echo 타임스탬프(`msg.timestamp`)를 비교. `IFF_ECHO` 플래그 확인 필수 |
| P2 송신 도구가 실제 모터 명령 ID(예: node12 Reset `0x196`)를 막지 못함 | ②③ | node 63(0x3F) 명령 비트만 허용(`probe_id`), 송신 전 2초 수신해 node 63이 보이면 중단 |
| P2 NVM 점검이 보드 누락을 통과 | ③ | `check(configs, expected)` + CLI `--expect` 필수, 누락·초과·중복 인자 검사 |
| P2(수렴 재검토) heartbeat를 끈 축이 node 63이면 사전 수신 검사로 못 찾고 `0x7F6` Reset이 그 보드에 도달 | 재검토 | NVM 점검이 STD node 63 예약·heartbeat 0을 위반으로 처리, 송신 도구는 세 보드 이상 PASS한 `--nvm-report`를 필수로 요구(`require_inventory`) |
| P3 `info_storm`이 알림 재활성화 누락 변이를 못 잡음 | ② | 폭주 뒤 LEC 1회가 계수되는지 추가 단언 |
| P3 Task 10 Step 4 기대값 모호, D5 요청→래치 지연 실측 누락, D2 32개 cmd 시험 누락 | ③ | Step 4에 guarded RESUME **거절** 기대·수동 재시작 경로·자동 수락 경로는 실물 미검증 명시, D5 지연 측정 추가, `filter_own_nodes`에 32 cmd × DATA/RTR 추가 |
| (자체 발견) Task 5·6 시험이 아직 없는 필드를 참조해 RED 단계에서 전체 컴파일 실패 | ① | 기능 매크로 `HAS_ATTEMPTS`·`HAS_D0`로 감쌈 |
| (자체 발견) 축 루프 대기에서 busy-spin 감지기를 리셋하지 않아 `d0_query_only`가 하네스 오류로 실패 | ① | `wait_for_current_meas` 스텁에서 리셋(Task 1 패치에 포함) |

검증: Task 1 패치를 깨끗한 체크아웃 사본에 적용해 patch3 `88/0`, 원본 `6/82`. Task 9 도구 시험 10개 통과, 실제 증거 JSON으로 NVM 점검 PASS. 두 검토 모두 기대 개수 산술 자체는 맞다고 확인했다(Task 1 시점 기준 +1 보정 반영: 95 → 104 → 107 → 111 → 128).

### 수렴

같은 스레드 재검토 2회 뒤 "새 P1/P2 없음 — 수렴"(2026-10-15). Codex가 계획의 코드를 메모리에서 합성 실행해 펌웨어·하네스 `128/0`, 도구 시험 `12/12`를 확인했다. 판정 범위는 계획·호스트 하네스이며 Cortex-M4 빌드와 실물 인수는 Task 7·10의 게이트로 남는다.

## 구현 검증 로그

2026-10-15 브랜치 `feat/mks-can-patch4`에서 Native 실행. 판단 기록 전체는 실행 원장(작업공간 `progress.md`)에 있고, 계획과 달라진 점은 다음과 같다.

- **하네스 충실도(Task 1, Codex P2×3).** 필터 모드·CAN1 bank 소유권, `do_checks`/`do_updates`의 오류 종료, NVIC 차단 중 CAN IRQ 미실행을 모형화하고 자체 시험 2개를 더했다. 이후 모든 개수가 +2.
- **Task 2·3(Codex 3라운드).** 이진 세마포어 모형과 "ErrorCode 읽기 직후 IRQ" 주입을 더하고, 반복 복구가 차단 없이 도는 경로를 막는 **시간 기반 백오프**(마지막 성공 복구 10 ms 안의 재복구는 `osDelay(10)`)를 추가했다. 시험 `info_atomic_take`·`info_repeat_recovery_yields`·`info_repeat_alternating_yields`. 필터 IDE 비교 변이를 잡도록 EXT `13<<23` 별칭 단언을 보강했다.
- **Task 4.** `pend_request_wakes_server`는 Task 2에서 이미 충족돼 처음부터 통과(가드로 유지).
- **Task 6.** Codex 1라운드 수렴, 시험 주석 정정(메서드 정책 시험).
- **순서.** Task 9를 Task 8보다 먼저 실행해 브랜치 전체 검토가 도구까지 다루게 했다.
- **최종 검토(서브에이전트 + Codex).** 수정 패스: D0 만료 시 입력 0(`d0_expiry_clears_input`); 측정 도구를 표준 라이브러리 raw CAN 소켓으로 재작성(STD `0x7E1` node 63 heartbeat 고정, `--cmd` 제거, `MSG_CONFIRM`+실행별 nonce로 자기 echo 판별, 버스 확인된 RTR 조회 뒤 ≥0.5 s 청취, 송신 중 node 63 등장·수신 종료 시 중단, 부분 실행 불합격, 인자 유한성 검사); NVM 점검 JSON에 보드 요약(`enable_watchdog` 포함); 기존 CAN 도구(motor_gui `odrive_can`, `can_drive_test.py`)의 D0 만료를 설계 §4-9·README·Task 10에 명시. Codex 같은 스레드 5라운드 뒤 수렴. 측정 도구는 로컬 vcan에서 정상·응답기·송신 중 등장·deadline 네 시나리오를 실제 실행했다.
- **최종 수치.** 하네스 개발본 `134/0`, patch3 음성 대조 `88/46`, 원본 `8/126`; `tools/tests` 통과; Cortex-M4 빌드 BIN 251,868 B(SHA256 `15462f7f…b3fc`), ELF 1,103,132 B, 변경 파일 경고 0.
- **Jetson 시험(2026-10-21).** Jetson(aarch64, Python 3.10, g++ 11.4)의 임시 디렉터리에서 하네스 패치본 `134/0`, 원본 `8/126`, 준비·재생성 도구, NVM 점검 7개, 측정 도구 단위시험이 통과했다. vcan에서 측정 도구를 실제로 돌리자 간격 0 송신에서 **자기 echo를 놓쳐 불합격**했다(수신 스레드가 느려 소켓 큐가 넘침). 수신 버퍼를 8 MiB로 키우고 `SO_RXQ_OVFL` 드롭 수를 `socket_drops`로 보고하게 고친 뒤 1,000개 3회와 컨테이너 20,000개가 통과했다. 같은 날 can0은 BUS-OFF(ACK 노드 없음), 수신 프레임 0, USB ODrive 없음 — 보드 전원이 꺼져 있어 실물 인수는 하지 못했다.
- **남은 것.** Task 10 실물 인수(사용자 단계별 승인)와 최종 검토의 Minor 7건(원장 `Final: minor (deferred)`).

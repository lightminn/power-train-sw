# ODrive CAN baud rate 통일 방법

- Claude.ai conversation UUID: `a1458e0b-a24d-4e3a-8cab-342c529ef6b5`
- Created: `2026-05-17T08:04:08.820578Z`
- Updated: `2026-05-17T08:11:43.073435Z`
- Categories: 파워트레인 관련 기록
- Source archive: `/home/light/Documents/카카오톡 받은 파일/data-af79641e-0ca7-4a6e-9a97-5140773682e0-1783641638-c1680b21-batch-0000.zip`

## Exported Summary

**Conversation Overview**

The person is working on a robotics or motion control project requiring simultaneous CAN bus communication with 10 motors — a mix of AK motors (CubeMars MIT protocol) and ODrive-MKS controllers. The core technical problem was a CAN baud rate mismatch: AK motors operate at 1Mbps while the ODrive-MKS firmware (`ODriveFirmware_v3.6-56V.elf`) appeared to have 250kbps baud rate hardcoded. The person wanted to unify all 10 nodes at 1Mbps and asked Claude to explore firmware modification or other methods to achieve this.

Claude initially corrected the premise, explaining that 250kbps is merely the default value in ODrive 0.5.x firmware (which the MKS version is based on), not a hardcoded constant, and advised using `odrv0.can.config.baud_rate = 1000000` followed by `odrv0.save_configuration()` via odrivetool over USB. The person attempted this and received a concrete error: `Exception: this attribute cannot be written to`, disproving Claude's initial assessment. Claude then revised its understanding, explaining that in ODrive 0.5.1 specifically, that attribute is read-only — the writable attribute was introduced in a later version when `can.set_baud_rate()` was removed and replaced with direct attribute writing. Claude then suggested trying `odrv0.can.set_baud_rate(1000000)` as the correct approach for the 0.5.1-era firmware, along with tab-completion inspection commands (`dir(odrv0.can)`) to verify available methods, and flagged the possibility that the MKS fork may have removed the baud rate change interface entirely. The conversation ended awaiting the person's test results of the method call approach.

Key technical context preserved: the firmware file is `ODriveFirmware_v3.6-56V.elf` from the `makerbase-mks/ODrive-MKS` GitHub repository; the ODrive hardware is v3.6; the firmware is based on ODrive v0.5.1; AK motors use 11-bit standard CAN IDs (MIT protocol); node ID collision between AK motors and ODrive CANSimple protocol was flagged as a concern for the 10-node setup.

## Conversation

### User · 2026-05-17T08:04:09.218229Z

지금 AK모터와 ODrive를 동시에 CAN통신으로 돌리려고 한다. 그런데 ODrive-MKS의 펌웨어(https://github.com/makerbase-mks/ODrive-MKS/blob/main/02_Makerbase%20ODrive%20related%20documents/ODriveFirmware_v3.6-56V.elf) 여기에 CAN baud rate가 250kbps로 하드코딩 되어있어서 AK모터의 1Mbps와 호환이 안된다. 모터 10개를 동시에 돌려야 하기 때문에 1Mbps로 통일하는게 좋다고 생각된다. 펌웨어 수정이나 기타 방법을 통해서 baud rate를 통일하는 방법을 탐구해봐라

### Claude · 2026-05-17T08:05:15.393194Z

사용자가 ODrive-MKS 펌웨어의 CAN baud rate가 250kbps로 고정되어 있어서 AK모터의 1Mbps 설정과 맞추고 싶어 하는군요. 먼저 ODrive-MKS 펌웨어에서 실제로 250kbps가 하드코딩되어 있는지, 그리고 baud rate를 변경할 수 있는 방법이 있는지 확인해야겠습니다.

ODrive 펌웨어는 오픈소스이고 보통 CAN baud rate는 설정 가능한 파라미터인데, MKS 버전에서 이 기능을 제한했을 가능성이 있으니 정확한 정보를 찾아봐야겠습니다.
```
This block is not supported on your current device yet.
```

 ODrive의 공식 펌웨어에서는 `<odrv>.can.config.baud_rate` 속성으로 baud rate를 동적으로 변경할 수 있다는 걸 알았다. 250kbps가 기본값일 뿐 하드코딩된 게 아니다. MKS 버전 펌웨어는 v3.6-56V용인데, 이건 ODrive v3.x 라인이고 0.5.x 펌웨어와는 다른 버전대인 것 같다.

사용자가 제시한 .elf 파일은 컴파일된 바이너리이고, 실제 소스 코드는 다른 곳에 있을 테니 MKS ODrive 펌웨어 저장소를 더 깊이 있게 살펴봐야겠다.
```
This block is not supported on your current device yet.
```

 So the standard ODrive firmware actually allows changing the baud rate from the default 250kbps to 1000000 through configuration, which means it's not hardcoded. Now I need to check whether the MKS version has disabled this capability, and the user only provided the .elf file to work with.MKS 펌웨어가 실제로 그 기능을 막아놨는지 확인해보자. ODrive 공식 펌웨어는 250kbps가 기본값일 뿐 변경 가능하다.
```
This block is not supported on your current device yet.
```

 I'm discovering that MKS's ODrive firmware is based directly on the official ODrive v0.5.1 release—they added the firmware file in a pull request and are essentially using it as-is. Now I'm looking at the CAN configuration properties available in ODrive 0.5.1, particularly the baud rate settings.

The user's assumption that 250kbps is hardcoded appears to be incorrect—it's actually just the default value. Let me verify this by checking the ODrive 0.5.1 documentation and changelog to confirm whether the baud rate is actually configurable.
```
This block is not supported on your current device yet.
```

 Now I'm realizing the user's assumption was incorrect. The MKS ODrive firmware is actually based on the official ODrive v0.5.1 release, which does support baud rate configuration through the CAN settings property.

The 250kbps isn't hardcoded—it's just the default value, and it persists after saving to non-volatile memory, though firmware updates will reset it back. The STM32F4 controller on the v3.6 board handles 1Mbps without issue, so switching to match the motor's native 1Mbps is a straightforward and sensible approach.

I should clarify this misconception directly, explain the one-line configuration command in odrivetool, and mention the important caveat about saving the configuration and the reset behavior after DFU updates.전제가 틀렸다. 250kbps는 하드코딩이 아니라 기본값(default)이다.

MKS ODrive 펌웨어(`ODriveFirmware_v3.6-56V.elf`)는 ODrive 공식 v0.5.1 펌웨어를 거의 그대로 사용한다 (저장소 PR #7에 `ODrive-fw-v0.5.1.zip` 원본이 들어 있음). ODrive 0.5.x부터는 baud rate가 runtime configurable parameter이고, `odrv0.can.config.baud_rate`에 값을 쓰면 즉시 반영된다.

## 변경 방법

odrivetool에서 USB로 연결한 뒤:

```python
odrv0.can.config.baud_rate = 1000000
odrv0.save_configuration()   # NVM에 저장, 자동 reboot
```

이게 전부다. 펌웨어 수정도, elf 디스어셈블도 필요 없음. ODrive 0.5.x 펌웨어는 1Mbps, 500k, 250k, 125k, 50k, 25k를 모두 지원한다.

## 주의할 점 몇 가지

**DFU(펌웨어 업데이트) 후 config가 지워진다.** 0.5.1 DFU는 NVM을 함께 erase한다. DFU 직후엔 250kbps로 돌아가므로, USB로 다시 붙어서 1Mbps로 재설정해야 한다. 10개 보드면 이 작업이 좀 귀찮으니 backup/restore 스크립트(`odrivetool backup-config`, `restore-config`)를 미리 만들어두는 게 좋다.

**호스트 측 baud rate 불일치 시 BUS-OFF.** ODrive를 1Mbps로 바꾼 직후 host CAN 인터페이스가 250k면 즉시 BUS-OFF로 떨어진다. ODrive 설정을 바꾸기 직전에 host도 1Mbps로 미리 올려놓는다.

**Termination/배선 점검.** 10노드 + 1Mbps면 stub 길이, twisted pair, 양 끝단 120Ω 종단저항이 더 민감해진다. ODrive v3.6은 DIP 스위치로 종단저항 on/off 가능 — 버스의 양 끝 두 노드만 켜고 나머지는 끈다.

**AK 모터 ID와 ODrive node_id 충돌 확인.** AK 모터(CubeMars MIT 프로토콜)는 11-bit standard ID를 쓰고, ODrive CANSimple도 11-bit standard ID(상위 6비트가 node_id)다. 두 프로토콜이 ID 공간을 공유하므로 충돌 안 나게 할당해야 한다.

elf 안에 250000 상수가 있는 건 그냥 default 값이 컴파일된 거고, 그 값이 NVM의 사용자 config로 덮어쓰여진다. 펌웨어 빌드는 안 건드려도 된다.

### User · 2026-05-17T08:11:12.524706Z

In [2]: odrv0.can.config.baud_rate = 1000000
   ...:
---------------------------------------------------------------------------
Exception                                 Traceback (most recent call last)
Cell In[2], line 1
----> 1 odrv0.can.config.baud_rate = 1000000
File /usr/local/lib/python3.10/dist-packages/odrive/pyfibre/fibre/libfibre.py:674, in RemoteObject.__setattr__(self, key, value)
    672 if self.__sealed__ and not key in dir(self) and not hasattr(self, key):
    673     raise AttributeError("Attribute {} not found".format(key))
--> 674 object.__setattr__(self, key, value)
File /usr/local/lib/python3.10/dist-packages/odrive/pyfibre/fibre/libfibre.py:645, in RemoteAttribute.__set__(self, instance, val)
    643     return self._get_obj(instance).exchange(val)
    644 else:
--> 645     raise Exception("this attribute cannot be written to")
Exception: this attribute cannot be written to
In [3]:

안됨. 쓰기 불가라고 떠.

### Claude · 2026-05-17T08:11:43.073435Z

The user tried setting `odrv0.can.config.baud_rate = 1000000` but got a "cannot be written to" error, which is interesting since the official ODrive 0.5.1 firmware should allow writing to baud_rate. This suggests the MKS version might have made it read-only, which would explain why the user's attempt failed.

Looking at the changelog, I see that the automatic baud rate update feature was added around version 0.5.4-0.5.5, and the `can.set_baud_rate()` method was removed at that point. Since the user is on 0.5.1, they might need to use the older `can.set_baud_rate()` method instead of directly writing to the config attribute.
```
This block is not supported on your current device yet.
```

 I'm noticing something important in the changelog — `can.set_baud_rate()` was removed in a newer version, which means older versions like 0.5.1 required calling that method directly instead of writing to `can.config.baud_rate`. I need to figure out which version introduced this change.

Looking at the 0.5.1 documentation, it seems `axis.config.can.baud_rate` was read-only back then, so users had to call `odrv0.can.set_baud_rate()` with the desired baud rate value instead. This is likely the solution — I should suggest the user try this approach.원래 전제가 부분적으로 맞았다 — 0.5.1에선 그 attribute가 read-only다.

ODrive 공식 changelog에 "`<odrv>.can.set_baud_rate()` was removed. The baudrate is now automatically updated when writing to `<odrv>.can.config.baud_rate`"라는 항목이 있다. 즉, **이전 버전에서는 메소드 호출이 별도로 있었고, 나중에 그 메소드가 제거되면서 attribute 쓰기로 통합**된 거다. MKS가 쓰는 0.5.1은 그 변경 *이전* 버전이라 attribute가 read-only다.

먼저 이걸 시도:

```python
odrv0.can.set_baud_rate(1000000)
odrv0.save_configuration()
```

이게 통하면 끝. 안 통하면 메소드 이름을 확인:

```python
[m for m in dir(odrv0.can) if 'baud' in m.lower()]
```

또는 객체 트리 전체 탐색:

```python
odrv0.can      # tab completion 또는 그냥 출력해보면 사용 가능한 함수/속성이 다 보인다
```

만약 `set_baud_rate`도 없고 attribute도 막혀 있으면, MKS가 0.5.1 원본에서 baud rate 변경 인터페이스 자체를 빼버린 fork일 가능성이 있다 — 이건 직접 메소드/속성 확인해본 후에 판단 가능하다. 결과 알려주면 거기서 다음 단계 정한다.

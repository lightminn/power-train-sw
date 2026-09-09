# 망이 바뀌었을 때 — IP 지정해서 띄우는 법

IP 는 사람·공유기마다 다르다. 이 문서는 **자기 IP 로 스택을 띄우는 최소 절차**만 담는다.
코드에 박힌 기본값은 GL-SFT1200 AP(`ZETIN-ROBOT-5G`) 기준이며, 다른 망에서는 아래대로
**전부 인자로 넘기면 된다** — 소스를 고칠 필요 없다.

## 0. 먼저 두 IP 를 확인한다

| | 뜻 | 기본값 | 확인 방법 |
|---|---|---|---|
| **JETSON** | 로봇(젯슨)의 IPv4 | `192.168.8.106` | 젯슨에서 `hostname -I \| awk '{print $1}'` |
| **OPERATOR** | 운용 PC(노트북)의 IPv4 | `192.168.8.163` | 노트북에서 `hostname -I \| awk '{print $1}'` |

⚠️ 둘 다 **같은 서브넷**이어야 한다. 노트북이 유선·WiFi 양쪽에 붙어 있으면 젯슨과 같은
대역의 주소를 골라야 한다.

⚠️ `jetson-orin.local`(mDNS)은 IPv6 link-local 로 먼저 잡혀 SRT/UDP 가 안 붙는 경우가
있다. **문제가 생기면 이름 대신 IPv4 를 직접 넣는다.**

```bash
# 살아 있는지 먼저 확인 (노트북에서)
ping -c1 <JETSON>
ssh zetin@<JETSON> 'echo ok'
```

## 1. 젯슨 스택 기동 — 목적지는 접속한 PC 로 자동 설정된다

젯슨은 텔레메트리를 **운용 PC 로 UDP 송신**하므로 상대 주소를 알아야 한다.
`ssh -4` 로 접속하면 원클릭이 **지금 접속해 온 그 PC** 를 목적지로 자동 설정한다.

```bash
# 노트북에서 (SSH 접속 직후, 홈 ~)
ssh -4 zetin@<JETSON> 'bash ~/power-train-sw/scripts/jetson_gui_up.sh'
```

⚠️ `-4` 를 빼면 mDNS 가 IPv6 link-local 로 잡혀 운용 PC 의 IPv4 를 판별하지 못한다.
그때는 요약표에 `⚠️ OPERATOR_HOST 자동감지` 행이 뜬다 — `ssh -4` 로 다시 실행하면 된다.

데이터를 **다른 PC 로** 보낼 때만 주소를 직접 넘긴다:

```bash
ssh -4 zetin@<JETSON> 'bash ~/power-train-sw/scripts/jetson_gui_up.sh --operator-host <OPERATOR>'
```

⚠️ 목적지를 두 텔레메트리 서비스의 환경파일(`OPERATOR_HOST=`)에 반영하려면 root 권한이
필요하고, 젯슨 sudo 는 **항상 비밀번호를 요구**한다. 아래를 **1회** 실행해 두면 이후로는
비밀번호 없이 자동 반영된다. 안 해 두면 요약표에 `OPERATOR_HOST | 반영 실패` 가 뜨고
`:5004`/`:5005` 는 예전 주소로 계속 나간다:

```bash
# 젯슨에서 1회 (비밀번호 입력)
cd ~/power-train-sw && sudo bash scripts/install_operator_host_helper.sh
```

옵션:

```
--fresh            컨테이너 강제 재생성
--no-arm           로봇팔 스택 생략
--timeout SEC      헬스 대기(기본 420초)
```

✅ 기대: 파워트레인 컨테이너 health 통과 후 서비스 목록이 출력된다.

## 2. 운용 콘솔 — 젯슨 IP 를 넘긴다

```bash
# 노트북에서
/usr/bin/python3 -m operator_console.app --host <JETSON>
```

포트는 전부 기본값으로 충분하다(바꿀 일이 있을 때만 넘긴다):

```
--l515-port 5000            전방 영상(SRT)
--d435-port 5002            작업 카메라(SRT)
--metadata-port 5003        검출 좌표(UDP)
--telemetry-port 5004       파워트레인 텔레메트리(UDP)
--chassis-telemetry-port 5005
--arm-telemetry-port 5007   로봇팔 텔레메트리(UDP)
--ops-port 9001             조작 채널(TCP, 역할토큰)
--ops-host                  ops 브로커가 다른 호스트면 지정(기본: --host 와 동일)
--latency-ms 60             SRT 지연. 송신측 --srt-latency 와 같이 맞출 것
```

## 3. 영상만 볼 때 (콘솔 없이)

```bash
# 저지연 뷰어 — 원격주행용
./scripts/recv_stream.sh 5000 <JETSON> 60

# 좌표 오버레이 뷰어 — 정밀 접근·좌표 점검용(지연 더 큼)
python3 scripts/recv_yolo3d.py --host <JETSON>
```

## 4. 텔레메트리 서비스만 따로 설치할 때

설치 스크립트는 `OPERATOR_HOST` 를 환경변수 또는 인자로 받는다.

```bash
# 젯슨에서
OPERATOR_HOST=<OPERATOR> bash scripts/install_chassis_telemetry_service.sh
OPERATOR_HOST=<OPERATOR> bash scripts/install_pdist80b_telemetry_service.sh
```

## 5. 기본값을 아예 바꾸고 싶으면

운용 PC 주소 한 곳만 고치면 설치 스크립트들이 따라온다.

```bash
# scripts/operator_defaults.sh
DEFAULT_OPERATOR_HOST=<OPERATOR>
```

젯슨 주소의 기본값은 `operator_console/app.py` 의 `--host` 이며, 매번 인자로 넘기는 쪽을
권한다 — 소스 기본값을 고치면 다른 사람 환경에서 되돌려야 한다.

## 6. 안 붙을 때 보는 순서

| 증상 | 확인 |
|---|---|
| SSH 부터 안 됨 | `ping <JETSON>` → 다른 대역이면 노트북 인터페이스 확인 |
| 콘솔이 전부 회색(NO DATA) | 젯슨에서 `systemctl is-active` 로 서비스 확인, journal 무크래시 확인 |
| 텔레메트리만 안 옴 | 젯슨의 `OPERATOR_HOST` 가 지금 노트북 IP 인지 확인(1번을 다시 실행) |
| 영상만 안 옴 | SRT latency 를 송·수신 같은 값으로. 방화벽에서 해당 UDP 포트 확인 |
| 이름은 되는데 스트림이 안 붙음 | mDNS 대신 **IPv4 직접 지정** |

## 참고

- 살아 있는 젯슨 주소는 망에 따라 다르다. 과거 문서의 `192.168.50.98` 은 더 이상
  유효하지 않다(2026-08-02 확인).
- AP 설정과 자격증명은 `~/.claude` 환경변수로만 참조한다. 이 문서에 비밀번호를 적지 않는다.

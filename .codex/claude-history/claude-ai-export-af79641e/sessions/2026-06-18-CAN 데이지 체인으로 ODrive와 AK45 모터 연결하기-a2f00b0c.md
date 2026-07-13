# CAN 데이지 체인으로 ODrive와 AK45 모터 연결하기

- Claude.ai conversation UUID: `a2f00b0c-73dc-4711-aabb-e9a7471adadf`
- Created: `2026-06-18T18:53:33.890336Z`
- Updated: `2026-06-18T19:00:09.805353Z`
- Categories: 파워트레인 관련 기록
- Source archive: `/home/light/Documents/카카오톡 받은 파일/data-af79641e-0ca7-4a6e-9a97-5140773682e0-1783641638-c1680b21-batch-0000.zip`

## Exported Summary

**Conversation Overview**

The person is designing a 6-wheel Rocker-Bogie Rover and sought technical guidance on CAN bus wiring for the drivetrain system. The configuration consists of three ODrive motor controllers, four AK45-36 motors (two front, two rear for steering), one CAN transceiver module, and a Jetson Orin Nano, with the ODrives, Jetson, and transceiver housed inside the rover body and the AK motors mounted externally at the wheels.

The conversation focused on determining the correct CAN bus topology and physical wiring strategy. Claude recommended a strict linear (daisy-chain) topology with 120Ω termination resistors only at the two physical endpoints, and advised removing any built-in termination resistor from the transceiver module since it sits in the middle of the bus. The person proposed using the transceiver as a central hub with three branches going to the left AK motors, ODrive group, and right AK motors separately. Claude explained this would create a star topology, which violates CAN bus requirements due to signal reflections and termination impedance problems (three parallel 120Ω resistors yielding ~40Ω).

The person then clarified that ODrive CAN connectors are 2-pin (CAN_H/CAN_L only) with no pass-through capability, making T-branch splitter cables physically necessary at each ODrive connection point. Claude distinguished between acceptable T-stub drop connections (where the main bus line remains a continuous single line and each ODrive connects via a short stub) versus the problematic star topology the person had proposed. The correct approach keeps the main bus running as one unbroken line with ODrives and the transceiver each tapping in via short stubs (under 30cm), while AK45-36 motors use their 4-pin pass-through connectors inline without branching. Four T-splitters total are required: one per ODrive (×3) and one for the transceiver.

## Conversation

### User · 2026-06-18T18:53:34.346846Z

odrive 3개랑 AK45-36 모터를 CAN 통신 데이지 체인으로 연결해야 하는데 어떻게 할까? 트랜시버 모듈이랑 Jetson orin nano를 사용할 생각이고, 6륜 Rocker BOgie Rover에 사용할거야. AK motor는 앞바퀴 2개랑 뒷바퀴 2개에 조향용으로 들어갈거고, odrive 3개랑 Jetson, 트랜시버 모듈은 모두 Body 내부에 위치할거야. 어떻게 결선해야할까

### Claude · 2026-06-18T18:54:35.636046Z

사용자가 CAN 버스 데이지 체인 결선에 대해 물어보고 있는데, 이미 확정된 구조가 있다. 좌측 AK①에서 시작해서 AK②, 트랜시버, ODrive 세 개, 우측 AK⑥과 ⑦로 이어지는 순서로 연결되며, 양 끝(좌AK①과 우AK⑦)에만 120Ω 종단 저항을 두고 트랜시버 모듈 내부의 저항은 제거해야 한다.

실제 구성은 AK45-36 모터 4개(앞 2, 뒤 2), ODrive 3개, 트랜시버 1개로 이루어져 있으며, AK45-36은 4핀 패스스루 구조라 추가 분기가 필요 없다. CAN 버스는 데이지 체인으로 단일 연결되고, RS485는 드라이버 제어용으로 별도 운영된다. 그래서 CAN 버스 배선 구성은 데이지 체인 순서대로 정리하면 되는데, 양 끝(좌AK①과 우AK⑦)에만 120Ω 종단저항을 달고, 트랜시버 모듈에 내장된 종단저항은 제거해야 한다. T 분기는 ODrive 3개와 트랜시버 모듈 4곳에만 필요하고, AK 모터들은 패스스루 방식이라 분기가 필요 없다. CAN_H, CAN_L, GND는 모든 기기에서 공통으로 연결된다. Jetson Orin Nano의 40핀 헤더에 CAN 컨트롤러 핀이 노출되어 있고 SocketCAN으로 사용할 수 있는데, SN65HVD230 같은 외장 트랜시버 모듈이 필요하다. 이 모듈들은 보통 120Ω 종단저항이 내장되어 있어서 네트워크의 양 끝에만 필요한 저항을 제거해야 한다. 이제 토폴로지 순서와 양 끝 종단, 트랜시버 구성을 설명하면 될 것 같다. 간단한 텍스트 체인으로 결선 구조를 표현하는 게 낫겠다. 사용자가 원하는 간결함과 결선도의 필요성 사이에서 한 줄짜리 다이어그램이 적절한 균형점이 될 것 같다. CAN은 라인 토폴로지로 일렬 데이지 체인 구성이어야 하고, 차체 배치 순서대로 좌측 앞뒤 AK 2개 → 트랜시버 → ODrive 3개 → 우측 AK 2개로 연결된다. 종단저항 120Ω은 물리적 양 끝인 좌AK①과 우AK⑦에만 붙이고, 트랜시버 모듈의 내장 120Ω은 반드시 제거해야 한다. 중간 노드에 종단저항이 있으면 양 끝 2개와 트랜시버 1개가 병렬로 연결되어 약 40Ω이 되어 신호가 손상된다.

배선은 CAN_H, CAN_L, GND 3선을 공통으로 사용하되, 각 노드마다 짧은 스터브로 분기한다. ODrive는 CAN 핀이 패스스루 기능이 없어서 T 분기 스플리터가 필요하고, AK45-36은 입출력 패스스루 커넥터가 있어서 다음 모터로 바로 이어주면 된다. 모든 노드의 CAN GND를 공통 기준 전위로 묶어야 한다.

CAN 1Mbps 속도에서는 스터브 길이를 30cm 이내로 유지하되, 가능하면 더 짧게 하는 게 좋다.CAN은 line 토폴로지라 일렬 데이지 체인만 가능하다. 차체 배치(AK는 외곽, ODrive·Jetson은 body 내부) 고려하면 좌측 AK 2개에서 시작해 트랜시버 거쳐 ODrive 3개 지나 우측 AK 2개로 끝나는 순서가 배선 거리상 합리적이다.

```
[120Ω] 좌AK① ─ 좌AK② ─ 트랜시버(Jetson) ─ ODrive③ ─ ODrive④ ─ ODrive⑤ ─ 우AK⑥ ─ 우AK⑦ [120Ω]
```

핵심 세 가지.

종단저항 120Ω은 물리적 양 끝 두 노드에만 단다. 여기선 좌AK①과 우AK⑦. AK45-36은 내장 종단 설정 기능이 없으니(CubeMars 매뉴얼 확인했듯 CAN 설정은 ID·통신속도·timeout 처리뿐) 양 끝 AK에는 외부 120Ω을 직접 붙여야 한다.

트랜시버 모듈에 내장 120Ω이 있으면 반드시 제거해라(점퍼 또는 SMD). 트랜시버는 체인 중간 노드라 종단이 들어가면 안 된다. 그대로 두면 양 끝 2개 + 트랜시버 = 3개 병렬로 약 40Ω이 잡혀서 신호 진폭이 떨어진다.

배선은 CAN_H, CAN_L, GND 3선 공통이다. ODrive는 CAN 핀이 패스스루가 안 되니 ODrive 3개 + 트랜시버 = 4개 지점에 T 분기가 필요하다. AK45-36은 4핀 커넥터가 in/out 패스스루라 분기 없이 다음 모터로 그대로 이어주면 된다. 스터브는 짧게 — 1Mbps 쓸 거면 스터브당 30cm 이내, 가능한 더 짧게.

GND는 전 노드 공통으로 묶어 기준 전위를 맞춘다. ODrive·AK 모두 48V 버스 공유라 자연히 공통되겠지만, 트랜시버 GND를 Jetson과 버스 양쪽에 확실히 물려라.

### User · 2026-06-18T18:56:29.164989Z

그냥 트랜시버에서 3개 분기해서 우측 AK 쪽도 따로 처리하면 안되나

### Claude · 2026-06-18T18:57:07.514061Z

사용자가 트랜시버를 중심 노드로 해서 세 갈래로 분기하는 구조를 제안하고 있다. 좌측 AK 그룹, ODrive 그룹, 우측 AK 그룹을 각각 트랜시버에서 직접 연결하는 방식이 가능한지 묻는 것 같다.

이건 CAN 버스 토폴로지 원칙을 위반한다. CAN은 반드시 선형(line) 토폴로지여야 하고, 스타나 트리 구조는 안 된다. 임피던스 매칭이 핵심인데, 선형 구조에서 양 끝에 120Ω 종단을 달면 약 60Ω의 차동 임피던스가 유지되지만, 분기 구조로 가면 각 가지가 스텁이 되어 신호 반사가 발생한다. 게다가 종단을 어디에 둘지 불명확해지고, 우측 AK까지 긴 가지를 빼면 스텁 길이 제한을 초과해서 신호 무결성이 손상된다. 실제로는 가능하다. 우측 AK 배선 문제를 해결하려면 ODrive 끝에서 우측 AK로 한 줄을 빼면 되는데, 이건 이미 제안한 구조고 단지 한 가닥이 더 길어질 뿐이다. 또는 토폴로지 순서를 바꿔서 트랜시버를 한쪽 끝으로 보내고 좌AK→ODrive→우AK를 일렬로 연결한 후 트랜시버에 종단을 달 수도 있다. 결국 line 구조만 유지하면 토폴로지 순서는 자유롭다.

더 확실하게 분리하고 싶다면 CAN을 2채널로 운영하는 방법도 있다. Jetson Orin이 여러 CAN 컨트롤러를 지원하고 트랜시버 2개를 쓰면 독립적인 2개 버스를 만들 수 있다. 예를 들어 버스1에 좌AK와 ODrive, 버스2에 우AK를 배치하고 각 버스의 양 끝에 종단을 달면 된다. 다만 트랜시버 모듈과 CAN 채널이 추가로 필요하다.

사용자가 묻는 건 단일 트랜시버에서 3분기인데, 이건 전기적으로 하나의 버스면서 star 토폴로지가 되어 불가능하다. CAN은 단일 라인이어야 하고 트랜시버에서 분기하면 stub 반사와 종단 처리 문제가 생긴다. 종단을 3개 끝점에 다 달면 3개가 병렬로 연결되어 임피던스 문제가 생기고, 일부만 달면 반사가 발생한다.

정말 분리하려면 CAN 2채널로 트랜시버를 2개 써야 하는데, 그럼 단일 버스 설계를 바꿔야 한다.안 된다. 그건 트랜시버를 중심으로 하는 star/tree 토폴로지가 된다. CAN은 single line이어야 한다.

이유 두 가지다.

분기하면 각 가지가 stub이 되고 stub 끝에서 신호 반사가 생긴다. 1Mbps 쓰면 특히 치명적이다.

종단을 둘 데가 없어진다. line이면 끝점이 둘이라 양 끝에 120Ω 깔끔하게 맞지만, 트랜시버에서 3분기하면 끝점이 좌AK·ODrive열·우AK 셋이다. 셋 다 종단 달면 3 병렬 ~40Ω로 너무 낮아지고, 일부만 달면 종단 안 된 가지가 반사한다. 어느 쪽이든 깨진다.

우측 AK 배선이 멀어서 따로 빼고 싶은 거라면, 어차피 line에서도 한 가닥 더 길어지는 것뿐이라 별 차이 없다. 그리고 노드 순서는 전기적으로 자유라 배선 편한 대로 바꿔도 된다. 예를 들어 트랜시버를 끝으로 보내고

```
[120Ω] 트랜시버 ─ 좌AK ─ 좌AK ─ ODrive ─ ODrive ─ ODrive ─ 우AK ─ 우AK [120Ω]
```

이렇게 해도 무방하다. line만 유지하면 된다.

정말 좌우를 물리적으로 분리하고 싶으면 방법은 하나뿐이다 — CAN을 2채널로 쓰는 것. 트랜시버 2개 달아서 독립 버스 둘로 나누고(예: 버스1 = 좌AK+ODrive, 버스2 = 우AK) 각 버스를 양 끝 종단한다. 근데 이건 단일 버스 설계를 바꾸는 거고 Jetson CAN 채널·트랜시버가 하나 더 필요하다. 단일 트랜시버에서 갈래만 치는 건 안 된다.

### User · 2026-06-18T18:59:14.425534Z

odrive can 케이블이 2p 짜리라 분기 케이블이 필수임

### Claude · 2026-06-18T19:00:09.805353Z

사용자가 ODrive CAN 케이블이 2핀이라고 언급했는데, 먼저 이게 맞는지 확인해야 한다. MKS ODrive v3.6의 CAN 커넥터 사양을 정확히 파악해야 하는데, GPIO 핀을 사용하는 방식인지 확인 중이다.

사용자의 주장이 맞다면 ODrive는 CAN_H와 CAN_L 두 핀만 있어서 패스스루가 불가능하고, 따라서 데이지 체인을 위해서는 각 지점에서 T 분기가 필수라는 뜻이다. 이미 메모리에 "ODrive 3개 + 트랜시버 = T 분기 4개 필요"라고 기록되어 있으니 이건 확정된 사항이다.

직전 대화에서 사용자가 트랜시버에서 3분기하는 방식을 제안했고 내가 star 토폴로지 때문에 불가능하다고 답했는데, 지금 이 언급은 그 맥락에서 나온 것 같다. 이제 T 분기와 star 토폴로지의 차이를 명확히 해야 한다. T 분기는 메인 버스가 일직선으로 계속 이어지고 각 노드가 짧은 stub으로 연결되는 방식인데, ODrive가 2핀이라 패스스루가 불가능하니까 T 커넥터로 메인 버스에서 짧게 따서 연결하는 것이다. 반면 star 토폴로지는 한 중심점에서 모든 노드가 나가는 구조로, 이건 CAN 버스에서 권장되지 않는다.

사용자의 지적이 맞다. T 분기 케이블 자체는 문제가 아니고, ODrive처럼 패스스루가 안 되는 노드에 짧은 stub으로 붙이는 것이 정상이다. 문제는 그 분기를 버스 세그먼트 전체를 가르는 데 쓰는 것인데, 트랜시버에서 좌측 AK열, ODrive열, 우측 AK열을 각각 따면 그건 긴 세 가지가 되어 star 토폴로지가 되는 것이다.

핵심은 메인 버스를 한 줄로 유지하면서 각 노드마다 T 스플리터로 짧은 stub을 붙이는 건 완전히 괜찮다는 것. 하지만 중심에서 긴 가지들이 분기되는 star 구조는 피해야 한다. 이제 실제 결선을 구체적으로 설명해야 하는데, ODrive가 2핀(CAN_H, CAN_L)이고 패스스루가 안 되므로...

메인 버스 라인을 따라가다가 각 ODrive 위치에서 T 커넥터로 메인 라인은 통과시키고 짧은 가지를 ODrive 2핀에 연결하는 방식이다. 이게 바로 T 분기 커넥터가 필요한 이유고, 4개가 필요했던 것도 이 때문. 핵심은 stub 길이를 최대한 짧게 유지하는 것.

ODrive가 CAN_H/CAN_L 2핀만 있으면 GND는 별도로 필요 없다. ODrive의 전원 GND가 48V 버스와 공유되니까 시스템 공통 GND가 이미 있고, CAN 트랜시버는 그 공통 GND를 기준으로 동작한다.

정리하면, T 분기 스플리터가 필수인 건 맞지만 이건 stub 분기와 star 토폴로지를 구분하는 다른 문제다.

T 분기는 메인 버스가 한 줄로 계속 지나가고 ODrive는 짧은 stub으로만 붙으면 되는데, star 토폴로지처럼 메인 버스 자체를 여러 갈래로 나누는 건 피해야 한다. 각 ODrive마다 T 커넥터를 인라인으로 끼우고 짧은 케이블로 연결하는 게 정상적인 구성이다.맞다. ODrive는 CAN_H/CAN_L 2핀뿐이라 패스스루가 안 되니 T 분기 스플리터가 필수다. 그래서 ODrive 3개 + 트랜시버 = 4지점에 T 분기가 필요하다.

근데 이게 직전에 말한 "트랜시버 3분기"랑 같은 얘기는 아니다. **T 분기(stub drop)와 star 토폴로지는 다르다.**

T 분기는 메인 버스 라인이 좌→우로 한 줄 그대로 지나가고, ODrive는 그 라인에서 짧은 stub으로 따서 붙는 거다. 메인 line은 끊기지 않고 연속이다. stub만 짧으면(가능한 한 짧게, 길어도 30cm 이내) 전혀 문제없다. 오히려 ODrive 같은 비패스스루 노드는 이렇게 붙일 수밖에 없다.

반면 트랜시버 3분기는 메인 버스 자체를 세 세그먼트로 갈라 중심에서 뻗는 거라 연속된 line이 없다 = star. ODrive에 T 커넥터 쓴다고 이게 정당화되진 않는다. 둘은 별개다.

실제 결선은 이렇다.

```
메인 버스 (한 줄로 연속, 양 끝 120Ω)
[120Ω]좌AK ─ 좌AK ─[T]─[T]─[T]─ 우AK ─ 우AK[120Ω]
              트│  O│  O│
              랜│  D│  D...
              시│  r│  
              버   ive
```

ODrive와 트랜시버는 각자 위치의 T 커넥터에서 짧은 2핀 stub으로 메인 라인에 drop한다. AK는 4핀 패스스루라 T 없이 인라인으로 그대로 통과시킨다. 핵심은 메인 line의 연속성을 유지하는 것이고, T 분기는 그 위에서 짧게 따는 한 자유롭게 써도 된다.

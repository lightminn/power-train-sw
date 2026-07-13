# Jetson과 isolated CAN transceiver 배선

- Claude.ai conversation UUID: `bcd56514-4f44-44f0-b1cb-1082b2b0649a`
- Created: `2026-06-19T18:18:26.438700Z`
- Updated: `2026-06-19T20:03:27.746283Z`
- Categories: 파워트레인 관련 기록
- Source archive: `/home/light/Documents/카카오톡 받은 파일/data-af79641e-0ca7-4a6e-9a97-5140773682e0-1783641638-c1680b21-batch-0000.zip`

## Exported Summary

**Conversation Overview**

The person is building a multi-node CAN bus system using a Jetson Orin Nano connected to AK motors and ODrive-MKS motor drivers (10 nodes total, 1Mbps), and sought help wiring an isolated CAN transceiver to the Jetson. They purchased the MikroElektronika CAN Isolator click board (ADM3053-based) from a Korean electronics supplier.

Claude walked through the full wiring architecture across multiple clarifying exchanges. Key points established: the Jetson Orin Nano's CAN signals appear on a dedicated CAN header (not the 40-pin GPIO header), the ADM3053 click has an integrated isolated DC-DC converter meaning no external bus-side power supply is needed, and the logic-side (Jetson domain) wiring requires +5V to pin 10, +3.3V to pin 7 with JP1 set to 3V3, GND to pins 8/9, CAN_TX to TXD (pin 13), and CAN_RX to RXD (pin 14). The person asked whether Jetson 5V with only GND separated would work for the bus-side power — Claude clarified this is incorrect because voltage and GND must be treated as a pair, and connecting Jetson 5V to the bus side breaks isolation regardless of GND separation.

For the bus-side DB9 connection, the person asked whether CAN_GND (pin 3) should connect to ODrive or directly to the battery negative. Claude recommended the ODrive CAN connector GND rather than the battery negative, to avoid ground bounce from high motor currents corrupting the signal reference. The ADM3053 click's built-in split termination (R2+R3, each 60.4Ω in series = 120Ω total) means no external termination resistor is needed, but the board must be placed at one physical end of the bus, with only one other end node having termination enabled. Since the person is hand-making their own CAN cable, Claude specified using shielded twisted pair for CANH/CANL plus one CAN_GND wire, shield grounded at the Jetson/ADM3053 end only to avoid ground loops, daisy-chain topology with stubs under 20–30cm, and verifying DB9 pin assignments (standard CiA: pin 7=CANH, pin 2=CANL, pin 3=GND) with a multimeter before soldering since CANH/CANL reversal causes complete communication failure.

## Conversation

### User · 2026-06-19T18:18:27.285508Z

isolated can transeiver를 사서 jetson하고 연결하려 하는데, 배선 어떻게해

### Claude · 2026-06-19T18:20:06.619916Z

사용자가 Jetson Orin Nano에 isolated CAN transceiver를 연결하는 배선 방법을 묻고 있는데, 10개 노드의 CAN 버스(AK 모터와 ODrive-MKS)를 1Mbps로 운영하려고 한다. 정확한 핀아웃을 제공하려면 먼저 사용 중인 구체적인 transceiver 제품을 파악해야 하고, 다이어그램이 도움될 것 같다.

Jetson Orin Nano 모듈 자체에는 MTTCAN 컨트롤러가 내장되어 있어서 외장 transceiver만 필요한데, 개발자 키트의 40핀 헤더에 CAN 핀이 실제로 노출되는지 확인해야 한다. 개발자 키트 캐리어보드(P3768)의 핀 매핑을 살펴보니 Pin 29와 Pin 31에 CAN0_DIN과 CAN0_DOUT이 있는 것으로 보인다.

사용자가 구체적인 핀아웃을 알려면 정확한 CAN 핀 위치와 사용 중인 isolated transceiver 제품 사양이 필요하다. Isolated transceiver는 컨트롤러 쪽(VCC, GND, CAN_TX, CAN_RX)과 버스 쪽(별도 전원, GND_iso, CANH, CANL)에 각각 독립적인 전원이 필요하다는 게 핵심이다. 로직 측 전원과 버스 측 전원을 분리해야 isolation이 제대로 작동하고, 버스 양 끝에 120Ω 종단 저항을 달아야 하며, CAN_TX/CAN_RX를 transceiver의 D/R 핀에 연결해야 한다. 정확한 배선을 위해서는 사용할 transceiver 모듈이 무엇인지, 그리고 Jetson Orin Nano dev kit인지 확인이 필요하다. 정확한 제품 정보 없이는 핀아웃을 특정할 수 없으니, 먼저 일반적인 isolated CAN transceiver 구성 원리를 설명하고 Jetson Orin Nano의 CAN 핀 배치를 확인한 후 사용자에게 transceiver 제품 하나만 물어보는 게 낫겠다. 사용자가 간결한 답변을 원하는 만큼 핵심만 전달해야 한다. 일단 검색으로 정확한 핀아웃을 확인한 후 isolated transceiver 배선 방법을 설명하고, 마지막에 사용자의 구체적인 제품 정보를 물어봐야겠다. 한국어 평어체로 답하자.Jetson Orin Nano의 정확한 CAN 핀 위치부터 확인할게.
```
This block is not supported on your current device yet.
```

중요한 발견이 있다: Orin Nano 개발 키트의 CAN 버스가 40핀 헤더가 아니라 별도의 선택적 CAN 버스 헤더에 위치한다는 것을 carrier board 스펙에서 확인했다. 정확한 핀아웃을 위해 JetsonHacks의 Orin Nano 문서를 확인해야 한다.Orin Nano dev kit는 CAN이 40핀이 아니라 별도 CAN 헤더에 있어. 정확한 위치를 확인할게.
```
This block is not supported on your current device yet.
```

CAN isn't on the 40-pin J12 header—it's a separate connector. The carrier board spec mentions an optional CAN bus header, and the Orin Nano dev kit has its own dedicated CAN header (J17 or similar) with an onboard MTTCAN controller.

The CAN signals (TX/DOUT, RX/DIN, 3.3V, GND, and possibly +5V) come out on this separate header as raw 3.3V logic signals without a transceiver. So I need an isolated transceiver to bridge between the Jetson's 3.3V CAN signals and the external bus—connecting the transceiver's logic side (VCC, GND, TXD, RXD) to the Jetson's CAN header, making sure the transceiver's logic level matches the 3.3V standard.

On the bus side, the transceiver connects CANH and CANL to the CAN network, but isolation only works if the bus side has its own separate power supply (VISO). This can come from either an integrated isolated DC-DC module built into the transceiver (like ADM3053 or some Waveshare modules) where I only need to power the logic side, or I need to provide a separate isolated power source for the bus side if the transceiver doesn't have one built in. Either way, I'll need a 120Ω termination resistor on the bus.

The critical part is keeping the logic ground (Jetson side) completely isolated from the bus ground (motor side) — that's the whole point of isolation. Only CANH, CANL, and optionally CAN_GND should connect to the bus side. Since I don't know which specific transceiver module the user has, I need to ask what product they're using and whether it has an integrated DC-DC converter, because the pinout varies significantly between different modules. 텍스트로 핵심을 먼저 정리하고, transceiver 모델이 확정되면 정확한 핀 연결도를 제안하는 게 낫겠다. Orin Nano는 CAN 컨트롤러가 내장되어 있지만 transceiver는 없어서 별도로 준비한 게 맞고, 신호는 40핀이 아니라 별도 CAN 헤더를 쓴다. isolated transceiver 배선의 핵심은 logic side와 bus side를 분리하는 것인데, GND를 나누고 bus side에 별도 전원을 공급해야 하며, termination은 120Ω을 써야 한다. 마지막으로 어떤 transceiver 모델을 쓰는지 물어봐야 DC-DC 내장 여부를 판단할 수 있다.

Orin Nano dev kit의 CAN 헤더 정확한 핀 번호는 carrier board 스펙에 따라 다르고, 보통 버튼 헤더 근처에 있지만 정확한 매핑은 스펙 PDF를 확인해야 한다. 일반적으로 CAN_RX, CAN_TX, 3.3V, GND가 헤더에 있는 건 알려진 사실이다.

정확한 핀 번호를 모르니까 carrier board 스펙을 참고하라고 명확히 하고, 신호 이름으로 설명하되 추측을 사실처럼 말하지 않겠다.핵심부터. Orin Nano는 CAN controller(MTTCAN)를 내장하고 있지만 transceiver가 없다. 그래서 외장 transceiver 사는 건 맞는 선택이다. 그리고 CAN 신호는 40핀 헤더(J12)에 없다 — 별도 CAN 헤더에 나온다. 거기에 raw 3.3V 로직 신호로 CAN_TX(DOUT), CAN_RX(DIN), 3.3V, GND가 있다. 40핀에서 찾으면 안 된다.

배선은 transceiver를 기준으로 두 쪽으로 나뉜다.

**로직 쪽 (Jetson에 붙는 쪽)**
- Jetson CAN_TX → transceiver TXD(보통 `D`)
- Jetson CAN_RX → transceiver RXD(보통 `R`)
- Jetson 3.3V → transceiver 로직 VCC
- Jetson GND → transceiver 로직 GND

여기서 주의: transceiver 로직 쪽이 3.3V 입력을 받는지 확인해야 한다. 5V-only면 Jetson 3.3V 로직과 안 맞아서 레벨 시프트가 필요하다. ISO1042, SN65HVD230 격리판 같은 건 3.3V OK.

**버스 쪽 (격리된 쪽, 모터 bus에 붙는 쪽)**
- transceiver CANH → bus CANH
- transceiver CANL → bus CANL
- 버스 쪽 전원(VISO) → 여기가 갈린다 (아래 질문 참고)

**격리의 핵심 한 가지**: 로직 GND(Jetson)와 버스 GND(모터 시스템)를 절대 연결하지 마라. 연결하면 isolated 산 의미가 없어진다. 버스 쪽으로 넘기는 건 CANH/CANL뿐(+ 필요시 buffer 노드끼리 공유하는 CAN_GND). Jetson GND는 로직 쪽에만.

**termination**: Jetson이 bus 물리적 끝 노드면 CANH-CANL 사이 120Ω 필요하다. 중간 노드면 빼야 한다. 모듈에 점퍼로 내장된 경우 많으니 확인해라. 10-node bus니까 양 끝 2개만 120Ω, 나머지는 다 빼는 게 맞다.

baud는 `ip link set can0 type can bitrate 1000000`으로 맞추면 된다.

하나만 확인하자 — 산 transceiver가 **isolated DC-DC 내장형**인가, 아닌가? 내장형(ADM3053, 일부 Waveshare 격리 보드)이면 로직 쪽 전원만 주면 버스 쪽 전원이 자동 생성돼서 배선이 끝난다. 비내장형(ISO1042 단품 등)이면 버스 쪽에 별도 격리 전원을 따로 물려줘야 한다. 모델명 알려주면 정확한 핀-핀으로 짚어줄게.

### User · 2026-06-19T18:22:08.171632Z

버스쪽 5V하고 GND는 어느 전원에 연결해? 젯슨은 안되잖아

### Claude · 2026-06-19T18:23:05.685439Z

사용자가 핵심을 지적했다 — isolated transceiver의 버스 쪽 5V/GND를 Jetson에서 직접 따면 격리가 깨진다. 현재 시스템에 CubeMars 모터, ODrive-MKS, 18V rail, 48V 모터 전원이 있고, Jetson은 별도 전원을 쓰고 있는데, transceiver의 버스 전원을 어디서 가져와야 할지 고민 중이다.

핵심은 버스 쪽 GND가 CAN 노드들의 공통 GND가 되어야 한다는 것 — 격리의 목적이 Jetson 로직 도메인과 모터 파워 도메인을 분리하는 것이므로, 버스 전원은 모터/드라이버 쪽 도메인에서 가져와야 한다. ODrive나 MKS 보드가 GPIO 헤더에 5V를 제공하는지, 그리고 그 전류와 노이즈 특성이 transceiver 격리 쪽으로 충분한지 확인해야 하거나, 별도 isolated DC-DC를 고려해야 한다. 격리 transceiver를 사용하면 Jetson 쪽 GND와 독립적으로 버스가 자신의 도메인에서 동작할 수 있다. 버스의 전원과 GND는 두 가지 방식으로 구성할 수 있는데, 하나는 ODrive나 모터 컨트롤러 같은 CAN 노드에서 제공하는 전원 라인(보통 5V와 GND)을 버스 도메인의 기준으로 사용하는 것이고, 다른 하나는 별도의 격리된 DC-DC 컨버터를 사용하는 방법이다.

격리 DC-DC를 쓰면 Jetson의 5V를 받아 독립적인 5V를 생성하고, 그 출력 GND를 버스의 공통 기준점으로 삼아 CAN 노드들과 연결한다. 격리를 했더라도 버스 상의 노드들 간에는 공통 GND 레퍼런스가 필요할 수 있는데, 순수 차동 신호만으로도 동작하지만 common-mode 안정성을 위해서는 이런 기준점이 도움이 된다.

실제로는 모터 드라이버 도메인(48V 배터리 GND 기준)에서 5V를 가져오거나 별도 buck으로 생성한 후, 그 GND를 star ground로 삼는 게 가장 깔끔하다. 이렇게 하면 격리 회로가 정말로 보호해야 할 것—모터 파워 GND의 노이즈로부터 Jetson을 보호하면서도, 버스 쪽 transceiver는 모터 도메인에 속하게 되어 일관된 전원 구조를 유지할 수 있다. 실제로는 ODrive나 모터 컨트롤러에서 5V를 공급하거나 48V 배터리 기준의 별도 buck converter를 쓰는 게 일반적이고, 그 GND가 CAN 노드들의 공통 기준점이 되는데 Jetson 전원과는 절대 섞으면 안 된다. 깨끗한 5V 소스가 없으면 Jetson 5V에서 isolated DC-DC(B0505S 같은)를 거쳐 버스 쪽에 공급하면 격리를 유지하면서 Jetson 전원만으로 버스를 동작시킬 수 있다.

이 경우 DC-DC 출력 GND를 다른 CAN 노드들의 CAN_GND와 연결해서 공통 기준점을 만들어야 통신이 안정적이다. 다만 transceiver가 이미 isolated DC-DC를 내장하고 있다면 버스 쪽 전원을 따로 줄 필요가 없고, 로직 쪽 5V나 3.3V만 공급하면 내부에서 버스 쪽 격리 전원을 생성한다. 그래서 transceiver 사양을 먼저 확인하는 게 중요했던 것이다.

사용자가 정확한 모델을 알려주지 않았으니 두 경우를 모두 설명해야 하는데, 핵심은 버스 도메인의 전원이 어디서 나오는지에 따라 달라진다는 점이다. 그리고 CAN_GND 연결 시 격리되어 있어도 transceiver들 간의 공통 모드 기준이 필요하다는 점도 중요하다.

사용자의 star ground 구성을 보면 모터와 드라이버들이 모두 같은 GND 도메인(48V 배터리 음극)을 공유하고 있으므로, 버스 쪽 transceiver GND도 그 도메인에 연결하면 자동으로 기준이 잡힌다. 따라서 가장 실용적인 답은 버스 쪽 GND를 48V 배터리 음극(star ground)에 연결하고, 5V는 ODrive의 5V 출력이나 별도 buck 컨버터에서 가져오되, 이들이 모터 드라이버들과 같은 기준을 공유하도록 하는 것이다.버스 쪽 전원은 **모터/드라이버가 속한 도메인**에서 따와야 한다. 정확히는 너의 48V 배터리 negative(= star ground) 기준의 GND, 그리고 그 도메인에서 만든 5V.

이게 isolation의 핵심이다. 버스 쪽 transceiver는 모터 시스템 도메인에 "속해서" GND가 같이 출렁이고, Jetson은 격리벽 너머에서 안전하게 떨어져 있는 구조다. 그러니 버스 쪽 GND를 모터 드라이버들과 같은 reference에 물려야 CAN 통신 common-mode가 성립한다. 어차피 AK 모터랑 ODrive는 이미 그 GND를 공유하고 있으니, 버스 쪽 transceiver GND도 거기 붙이면 자동으로 reference가 잡힌다.

5V 소스는 둘 중 하나:
- ODrive(MKS) GPIO 헤더의 5V 출력을 뽑거나
- 그 도메인에서 별도 buck으로 5V 생성 (네 18V→5V buck 라인을 하나 더 두는 식)

전류는 격리 transceiver 버스 쪽이 수십 mA 수준이라 부담 없다.

만약 버스 도메인에 줄 깨끗한 5V가 마땅찮으면 대안이 있다 — Jetson 5V를 받아서 **isolated DC-DC**(B0505S 같은 1W급)로 격리된 5V를 만들어 버스 쪽에 공급. 이러면 Jetson 전원만 쓰면서도 격리는 유지된다. 단 이때 DC-DC 출력 GND를 다른 CAN 노드 GND와 묶어 reference를 잡아줘야 한다.

근데 — 산 transceiver가 isolated DC-DC 내장형이면 이 고민 자체가 없다. 버스 쪽 전원 안 줘도 로직 쪽 5V만으로 내부에서 격리 전원이 생성된다. 그래서 모델명이 중요하다. 뭐 샀어?

### User · 2026-06-19T18:24:30.748588Z

5v를 젯슨에서 뽑고 그라운드만 분리시켜놔도 되냐?

### Claude · 2026-06-19T18:25:00.417978Z

사용자가 Jetson의 5V를 트랜시버에 공급하면서 GND만 모터 쪽 버스 도메인에 연결하려고 하는데, 이건 작동하지 않는다. 두 도메인 간에 공통 기준점이 없으면 신호 레벨을 제대로 해석할 수 없으므로 명확히 잘못된 접근이다.

전원은 V와 GND 쌍으로 정의되는데, Jetson의 5V는 Jetson GND 기준이다. 모터 GND를 분리하면 두 GND 사이의 전위차 때문에 그 5V가 모터 도메인에서는 5V가 아니게 된다. 게다가 모터 시스템의 GND는 고전류 스위칭으로 인해 계속 변동하므로 트랜시버의 공급전압이 불안정해져서 망가진다.

더 근본적으로, Jetson 5V 라인을 버스에 연결하는 순간 두 도메인이 전기적으로 연결되어 격리가 깨진다. 격리를 유지하려면 전원 도메인 전체(V와 GND 모두)를 분리해야 한다.

격리 트랜시버는 격리벽을 사이에 두고 로직 쪽과 버스 쪽이 각각 독립적인 전원 도메인을 가져야 한다. Jetson 5V를 버스 쪽 전원에 연결하면 격리의 의미가 사라지고, GND만 분리하는 건 반쪽 연결이라 전원 공급도 제대로 안 되고 격리도 안 된다.

올바른 방법은 버스 도메인에서 V와 GND 쌍을 모두 가져오거나, isolated DC-DC 컨버터로 Jetson 5V를 격리 변환해서 완전히 새로운 도메인을 만드는 것이다. B0505S 같은 격리형 DC-DC는 입력 도메인과 출력 도메인을 완전히 분리하기 때문에 이게 핵심이다.안 된다. 그건 격리를 절반만 끊는 거라 둘 다 망가진다.

이유 두 가지다.

첫째, 전원은 V랑 GND가 한 쌍이다. Jetson 5V는 "Jetson GND 기준 5V"라는 뜻이야. 그런데 GND를 모터 도메인에 물리면 그 5V의 기준점이 사라진다. 모터 GND가 Jetson GND보다 2V 떠 있으면 transceiver 입장에선 3V만 들어오는 거고, 모터 GND는 스위칭 때마다 출렁이니까 공급전압이 같이 흔들려서 transceiver가 오동작하거나 죽는다.

둘째, 더 중요한 건 — GND를 분리해도 **5V 라인 자체가 두 도메인을 연결**해버린다. 격리는 V랑 GND를 둘 다 끊어야 의미가 있다. 한쪽만 끊으면 나머지 한 줄로 그대로 이어지니까 산 의미가 없다.

핵심은 이거다. 격리 transceiver의 버스 쪽은 자기만의 독립된 전원 도메인(VISO, GND_ISO)을 가져야 한다. 그 도메인은 Jetson 도메인과 전기적으로 완전히 끊겨 있고, 신호만 격리벽을 통해 넘어간다. Jetson 5V를 끌어오는 순간 그 독립성이 깨진다.

Jetson 전원을 쓰고 싶으면 답은 **isolated DC-DC**다. B0505S 같은 거 쓰면 입력(Jetson 5V/GND) → 출력(완전히 분리된 5V/GND 쌍)을 만들어준다. V만 가져오는 게 아니라 격리된 전원 쌍을 통째로 새로 만드는 거야. 그 출력 쌍을 버스 쪽에 주면 격리 유지하면서 Jetson 전원으로 동작한다. GND만 분리하는 거랑은 완전히 다른 얘기다.

### User · 2026-06-19T18:28:42.760234Z

구체적으로 설명하셈

### Claude · 2026-06-19T18:30:18.968177Z

사용자가 isolated DC-DC 컨버터를 사용해서 Jetson 전원으로 버스를 구동하는 구체적인 방법을 원하고 있고, B0505S를 예시로 배선도를 그려주는 게 도움될 것 같다. 사용자가 모바일 환경이지만 텍스트로도 충분히 명확하게 설명할 수 있을 것 같다.

사용자가 "구체적으로"라고 명시했으니 전체 신호 흐름을 핀 단위로 정리해야 한다. B0505S와 격리 transceiver(ISO1042 같은 일반적인 모듈 기준)를 조합한 구성으로 설명하되, 사용자가 아직 transceiver 모델을 명시하지 않았으니 신호 이름 기준으로 일반화해서 접근하는 게 맞다. 또한 DC-DC 내장형 transceiver면 별도 DC-DC가 필요 없다는 점도 다시 강조해야 한다.

시스템 구조를 정리하면 Jetson 도메인(5V, 3.3V, GND)과 격리된 버스 도메인(VISO, GND_ISO)으로 나뉘는데, B0505S의 입력은 Jetson 5V와 GND에서 받고 출력이 격리된 전원을 만든다.

Transceiver의 로직 쪽은 Jetson의 3.3V(또는 5V) 전원과 GND, CAN_TX/RX 신호를 받고, 버스 쪽은 B0505S 출력의 VISO와 GND_ISO에서 전원을 받아 CANH/CANL 신호를 CAN 버스에 연결하는 구조다.

핵심은 GND_ISO를 모터 도메인의 GND와 연결해서 common-mode reference를 맞춰야 한다는 점인데, 이렇게 하면 Jetson과 모터 도메인 사이의 격리는 유지되면서도 CAN 통신이 제대로 동작한다.

그런데 생각해보니 B0505S를 쓰는 것과 그냥 모터 도메인에서 5V를 직접 가져오는 것의 차이가 뭔지 헷갈린다. 모터 도메인 전원을 쓰면 transceiver가 완전히 모터 도메인에 속하게 되고 별도 부품도 필요 없는데, B0505S를 쓰면 Jetson 5V를 격리해서 새로운 5V를 만드는 건데 결국 그 GND를 어디에 연결할지가 문제가 된다.

만약 GND를 모터 도메인에 연결하면 전원은 Jetson에서 오지만 기준점은 모터 도메인이 되는 상황이 생긴다. 하지만 B0505S가 입출력을 격리하니까 실제로는 동작한다 — Jetson 쪽 GND와 모터 쪽 GND가 분리되어 있고, 출력 5V는 모터 도메인 기준으로 정상적인 5V가 된다. 다만 이게 원래 의도한 "Jetson 5V를 쓰되 GND만 분리"하는 방식과는 다른 접근이다.

B0505S 방식의 정확한 배선을 정리하면, Jetson 쪽에서 5V와 GND를 공급하고, 트랜시버의 로직 전원도 Jetson 3.3V와 GND로 연결한 다음, CAN 신호들을 연결하는 구조다.

격리벽을 넘어서 버스 도메인에서는 B0505S의 출력이 트랜시버 버스쪽 전원과 GND_ISO를 공급하고, CAN 신호들이 버스에 연결되며, 핵심은 B0505S의 -Vout을 모터 도메인 GND에 연결할지 말지가 결정 포인트다.

격리 트랜시버와 격리 DC-DC를 함께 쓰면 버스 도메인이 완전히 독립적인 floating 상태가 되는데, CAN 통신이 제대로 되려면 이 floating 도메인이 다른 노드들의 common-mode 범위 안에 있어야 한다. 만약 AK나 ODrive 같은 다른 노드들이 비격리 트랜시버를 쓰고 모터 GND를 공유한다면, 우리의 GND_ISO를 그 모터 GND에 연결해서 common-mode를 맞춰줘야 하지 않으면 floating 도메인이 표류하게 된다.

결국 이렇게 하면 버스 쪽 GND가 모터 도메인 GND와 같아지고, 버스 쪽 5V는 B0505S가 모터 GND 기준으로 만드는 5V가 된다. 사용자 입장에서는 ODrive의 5V 출력이 부족하거나 노이즈가 심할 때 Jetson의 5V를 깨끗하게 격리해서 가져다 쓸 수 있다는 게 장점인데, 더 간단한 방법도 있을 것 같다. 그래서 B0505S 출력과 transceiver 버스가 모두 모터 도메인에 속하고, Jetson은 B0505S와 transceiver 내부 격리로만 분리되는 구조가 맞다. 사용자가 이해하기 쉽도록 배선도를 시각화해서 보여주는 게 가장 명확할 것 같다. 텍스트 설명과 간단한 다이어그램을 함께 제시하는 게 최선이겠다. SVG로 두 도메인을 박스로 구분하고 격리벽, 신호 흐름을 표현하려는데, 내 도구 목록에 visualizer 관련 도구가 명시적으로 없어서 tool_search로 찾아야 할 것 같다.

create_file로 SVG 아티팩트를 만들 수 있지만, 사용자가 원한 건 "구체적인 설명"이고 평어체와 간결함을 선호하는 것 같다. 텍스트 핀맵으로 충분히 명확하게 전달할 수 있으니, 구조화된 텍스트로 신호 흐름을 보여주는 게 더 적절할 것 같다.

B0505S 입출력 배선, transceiver 로직과 버스 배선, 그리고 GND_ISO를 모터 도메인 GND에 연결하는 부분이 핵심인데—특히 마지막 부분이 흔한 함정이다. Jetson의 5V를 격리해서 모터 도메인에 공급하는 방식을 명확하게 설명해야 한다. CAN 트랜시버 핀 배치는 모델마다 다르니까 신호 이름으로 설명하는 게 낫겠다. 로직 측 VCC, GND, TXD, RXD와 버스 측 VISO, GND_ISO, CANH, CANL을 기준으로 전체 배선도를 그려보자.

B0505S 아이솔레이션 모듈이 두 도메인을 분리하는데, 입력 쪽은 Jetson 5V와 GND를 받고 출력 쪽은 완전히 독립된 전원을 만든다. 그 출력 GND를 모터 도메인의 공통 기준점으로 연결해서 트랜시버가 버스의 다른 노드들과 같은 common-mode 레퍼런스를 공유하도록 해야 한다.

이렇게 하면 결국 Jetson의 5V를 격리 변환해서 모터 도메인 기준으로 공급하는 것인데, 이게 정확히 원래 의도한 "5V는 Jetson, GND는 분리"를 전기적으로 올바르게 구현한 방식이다. 다만 모터 도메인에서 5V를 직접 얻을 수 있다면 B0505S 자체가 필요 없어진다.

Jetson이 끝 노드면 CAN 종단 저항 120Ω을 붙여야 한다. 모델을 다시 물어볼 필요는 없을 것 같은데, 이미 충분히 설명했고 DC-DC가 내장되어 있으면 B0505S는 불필요하다는 점만 한 번 더 명확히 하고 작성을 시작하자.격리 DC-DC(B0505S 기준)로 가는 구성이다. 도메인이 두 개로 갈리고, 그 사이를 B0505S와 transceiver 내부 격리벽이 가른다.

배선은 이렇게 된다:

```
[Jetson 도메인]              |격리벽|        [모터 버스 도메인]

Jetson 5V    ──────────────→ B0505S  +Vin
Jetson GND   ──────────────→ B0505S  -Vin

Jetson 3.3V  ──→ TRX 로직 VCC
Jetson GND   ──→ TRX 로직 GND
Jetson CAN_TX──→ TRX TXD (D)
Jetson CAN_RX──→ TRX RXD (R)
                                      B0505S +Vout ──→ TRX 버스 VCC (VISO)
                                      B0505S -Vout ──→ TRX 버스 GND ──┐
                                      TRX CANH    ──→ bus CANH        │
                                      TRX CANL    ──→ bus CANL        │
                                                                      ↓
                                              모터 도메인 GND (48V 배터리 negative)
```

흐름을 풀면, B0505S 입력은 Jetson 도메인(5V/GND)에서 받는다. 출력(+Vout/-Vout)은 입력과 전기적으로 완전히 끊긴 새 5V 도메인이다. 이 출력 한 쌍을 transceiver 버스 쪽 전원으로 통째로 준다. V만 가져오는 게 아니라 격리된 V+GND 쌍을 새로 만들어 주는 거다 — 이게 네가 하려던 "5V는 Jetson, GND는 분리"를 전기적으로 올바르게 구현한 형태다.

빠뜨리면 안 되는 한 가지: B0505S 출력 GND(-Vout = transceiver 버스 GND)를 **모터 도메인 GND에 묶어라**. 이걸 안 하면 버스 쪽이 완전히 floating 상태가 돼서 common-mode가 표류하고, 결국 다른 노드(AK, ODrive)랑 통신이 안 된다. 격리 CAN에서 제일 흔한 실수다. AK랑 ODrive는 이미 48V 배터리 negative를 공유하니까 거기 물리면 된다. 네 star ground 기준점.

그러니까 정리하면 — B0505S를 거치는 순간 Jetson과의 격리는 유지되고(입출력 분리), 출력 도메인은 모터 쪽 GND에 reference를 잡는다. 결과적으로 "Jetson 전력으로 모터 도메인 기준 5V를 만들어 공급"하는 셈이다.

마지막 두 가지:
- 모터 도메인에서 깨끗한 5V를 직접 뽑을 수 있으면(ODrive 5V out 등) B0505S 자체가 필요 없다. 그게 제일 간단하다. B0505S는 모터 쪽 5V가 마땅찮을 때 쓰는 우회로다.
- 네가 산 transceiver가 isolated DC-DC 내장형이면 B0505S도, 버스 쪽 전원 배선도 전부 불필요하다. 로직 쪽 3.3V/5V만 주면 끝난다.
- Jetson이 bus 끝 노드면 버스 쪽 CANH–CANL 사이 120Ω 한 개.

### User · 2026-06-19T18:31:43.628340Z

https://m.vctec.co.kr/product/detail.html?product_no=11855&cate_no=161 이거임

### Claude · 2026-06-19T18:35:09.049448Z

사용자가 제공한 제품 URL을 가져와서 어떤 isolated CAN transceiver 제품인지 확인해야 한다. vctec.co.kr이라는 한국 부품 쇼핑몰 사이트인데, 모바일 버전 URL(m.vctec.co.kr)로 fetch를 시도하고 있다. 네트워크 설정의 allowed domains 확인이 필요한 상황이다.

web_fetch는 bash 네트워크와 독립적으로 작동하므로, 사용자가 직접 제공한 URL이나 검색 결과 URL을 가져올 수 있다. 일단 시도해보자.
```
This block is not supported on your current device yet.
```

MikroE CAN Isolator click uses the ADM3053, which has an integrated isolated DC-DC converter built in—so I don't need a separate power supply for the isolated side of the bus. That solves the power supply concern I had before.

The logic side just needs 3.3V or 5V from the Jetson, and the internal DC-DC converter generates the isolated bus-side power automatically. So the user's original idea about pulling 5V from the Jetson and only isolating GND isn't necessary—there's no need to supply the bus side at all.

Looking at the mikroBUS pinout: Pin 13 is TXD (Driver Input Data, where the Jetson's CAN_TX connects) and Pin 14 is RXD (Receiver Output Data, where the Jetson's CAN_RX connects). For power, the board can select between 3.3V and 5V via JP1, with both Pin 7 (+3.3V) and Pin 10 (+5V) available on the connector.

The ADM3053 itself runs on 5V as its main supply (4.5–5.5V range) to power the internal isolated DC-DC converter, but the logic I/O level is configurable through VIO to support either 3.3V or 5V compatibility. select the 3.3V option on JP1 to match Jetson's logic levels, then supply 5V to Pin 10, 3.3V to Pin 7, and connect GND on Pins 8 and 9. The key point is that the 5V and 3.3V supplied to the board are in the Jetson logic domain—the isolation happens internally within the board, while the CAN bus side gets its power from the internal DC-DC converter. 그래서 Jetson의 5V, 3.3V, GND를 모두 같은 도메인에서 공급하면 되고, ADM3053 칩이 내부적으로 격리를 처리하기 때문에 GND를 분리할 필요가 없다. 버스 쪽은 내부 DC-DC로 완전히 독립적으로 동작한다. 다 Jetson GND로 통일. 격리는 칩 내부 + DB9 버스 쪽에서 일어난다.

이게 이 보드의 장점이다 — "No need for an external supply for the isolated side". 

정정해야 할 점: 내가 이전 턴들에서 "버스 쪽 전원을 모터 도메인에서 따와라"라고 했는데, 이 보드는 DC-DC 내장형이라 그게 불필요하다. 첫 턴에 "DC-DC 내장형이면 버스 쪽 전원 안 줘도 된다"고 분기를 짚었으니 일관성은 있다. 이제 그 분기가 확정된 것.

ADM3053 칩이 DC-DC를 내장하고 있어서 버스 쪽 전원 공급 걱정이 사라진다. Jetson에서 5V, 3.3V, GND를 모두 공급하고 신호는 CAN_TX/RX를 Pin13/14에 연결하면 되는데, 격리는 칩 내부와 DB9 출력에서 자동으로 처리된다.

DB9 커넥터는 표준 CANopen 핀아웃을 따르므로 CANH(Pin7)와 CANL(Pin2)을 모터 버스에 직접 연결하면 된다. 모터 쪽이 단자나 와이어라면 DB9에서 신호를 뽑아내는 배선이 필요하다.

Termination 저항은 ADM3053 보드에 내장되어 있는지 확인해야 한다. 보통 click board는 120Ω을 점퍼로 선택 가능하거나 아예 없을 수 있으니 schematic을 봐야 정확하다. Jetson 노드가 버스의 끝이면 120Ω 저항이 필요하고, 모터 버스의 양 끝에 termination을 배치해야 한다.

격리된 버스 도메인의 GND(DB9 Pin3)를 어떻게 처리할지 생각 중이다.  ADM3053의 내장 DC-DC 때문에 격리측 GND(GND2)가 floating 상태가 되는데, 다른 노드들(AK, ODrive)과 CAN_GND를 연결해야 common-mode reference drift를 방지할 수 있다.

격리는 Jetson 도메인과 버스 도메인 사이에만 유지되고, 버스 측 GND2를 모터 도메인 GND에 연결하는 건 격리를 깨는 게 아니라 버스 도메인 내에서 안정적인 reference를 확보하는 것이다. 보호되고, 버스 통신은 모터 도메인 내에서 안정적으로 작동한다. 이제 최종 핀 연결을 정리하면, Jetson 쪽에서는 CAN_TX/RX를 TXD/RXD에 연결하고 전원과 GND를 공급하며, DB9 커넥터 쪽에서는 CANH/CANL을 버스에 연결하고 CAN_GND를 모터 도메인 GND에 연결한다.

Termination은 버스 끝 노드에 120Ω 저항이 필요하고, 클릭 보드 자체에 termination이 있는지 확인해야 한다. 사용자가 헷갈렸던 부분을 명확히 하면, 이 보드는 Jetson에서 5V와 3.3V, GND를 모두 함께 공급하고 칩이 내부적으로 격리를 처리하는 방식이며, GND 분리는 버스 측 GND를 모터 도메인에 연결하는 것을 의미한다.

배선 항목이 많으니 간단한 리스트로 정리하되, 사용자가 간결함을 선호하므로 명확하게 답변하되 다이어그램은 필요할 때만 제안하는 게 좋겠다. 다이어그램을 함께 제공하기로 결정했다. 사용자가 간결함을 원하지만, 핀 배선 같은 경우 시각화가 오히려 더 명확하게 전달할 수 있다. ADM3053의 전원 사양을 다시 확인 중인데, 5V 단일 전원 공급에 내부 DC-DC 변환기가 있고 VIO는 3.3~5V 범위를 지원한다.

Click board는 VDD1(DC-DC 입력)으로 반드시 5V가 필요하고, VIO는 JP1 점퍼로 3.3V 또는 5V 중 선택할 수 있다. Jetson의 로직이 3.3V이므로 JP1을 3V3로 설정하고 pin7에서 3.3V를 받으면서, 동시에 pin10에 5V를 연결해서 DC-DC 변환기에 전원을 공급해야 한다. Jetson 40핀 헤더에는 5V(pin2, 4)와 3.3V(pin1, 17)가 모두 있으니 이들을 활용하면 된다.

CAN 신호는 별도 CAN 헤더에 연결되고, 그 헤더의 전원도 같은 Jetson 도메인에서 가져올 수 있다. DB9 핀아웃은 CiA 303-1 표준을 기준으로 하되, ADM3053 click 보드의 schematic을 확인해서 실제 배치를 검증해야 한다. 이제 답변을 작성하겠다.ADM3053 click이네. 이거 핵심은 — **isolated DC-DC가 칩 안에 내장**돼 있다는 거다. 그래서 우리가 3턴 동안 고민한 "버스 쪽 5V 어디서 따냐"는 문제가 통째로 사라진다. 버스 쪽 전원은 외부에서 줄 게 없다. 칩이 내부에서 격리된 전원을 만들어낸다.

그러니 네가 물었던 "5V를 Jetson에서 뽑고 GND만 분리" — 이 보드에선 그렇게 하는 게 아니다. 로직 쪽 전원은 전부 Jetson에서 같이 주고, 격리는 칩이 알아서 한다.

**로직 쪽 (전부 Jetson 도메인, mikroBUS 핀 기준)**
- Jetson CAN_TX → TXD (pin 13)
- Jetson CAN_RX → RXD (pin 14)
- Jetson +5V → +5V (pin 10) ← 이건 내부 DC-DC 구동용이라 필수
- Jetson +3.3V → +3.3V (pin 7), JP1은 기본값 3V3(왼쪽)에 둬라. Jetson이 3.3V 로직이니까
- Jetson GND → GND (pin 8, 9)

여기 5V·3.3V·GND 전부 Jetson에서 따와도 된다. 격리벽이 칩 안에 있어서 이쪽은 그냥 Jetson 도메인 하나로 통일하면 된다.

**버스 쪽 (DB9 출력, 모터 도메인)**
- DB9 CANH → bus CANH
- DB9 CANL → bus CANL
- DB9 CAN_GND → 모터 도메인 GND (48V 배터리 negative, 네 star ground)

버스 쪽 전원은 연결 안 함. 단 하나 — **DB9의 CAN_GND는 모터 도메인 GND에 붙여라.** 격리측 GND2는 그냥 두면 floating이라 common-mode가 떠다닐 수 있다. AK/ODrive가 쓰는 GND에 물려서 reference를 잡아줘야 통신이 안정적이다. 이건 격리를 깨는 게 아니다. 격리벽은 Jetson ↔ 버스 사이에 있고, 버스측 GND를 모터 쪽에 붙이는 건 "버스 도메인 안에서" 기준 잡는 거라 Jetson은 여전히 보호된다.

DB9 핀번호는 표준 CANopen(CiA 303-1) 기준 CANH=7, CANL=2, GND=3인데, 이 click 보드가 그 표준 따르는지 schematic이나 멀티미터로 한 번 확인해라.
```
This block is not supported on your current device yet.
```

이제 핀 연결 다이어그램을 그려야 하는데, 좌측에 Jetson, 중앙에 ADM3053 격리 칩, 우측에 CAN 버스와 모터를 배치하고 각 도메인을 박스로 구분해서 연결선으로 표현할 거다. Writing SVG layout... 3열 구조로 재정렬하는 게 낫겠다. 왼쪽에 Jetson 도메인 박스에 5개 핀, 중앙에 ADM3053과 갈바닉 절연을 나타내는 수직 점선, 오른쪽에 버스 도메인 박스에 3개 핀을 배치하고, 도메인 간 흐름을 화살표 2개로 표현하되 핀 정보는 박스 내 텍스트로 담아서 절연벽을 핵심 메시지로 강조하자.

이제 SVG 레이아웃을 그려보는데, 좌측 파란 박스에 Jetson 도메인 제목과 CAN_TX, CAN_RX, +5V, +3.3V, GND 핀들을 배치하고, 중앙에 ADM3053 칩 박스를 놓으면서...

ADM3053 박스 내부에 갈바닉 절연을 나타내는 빨간 수직 점선을 그어서 좌측(로직 사이드)과 우측(CAN 버스 사이드)을 분리하고, 각 쪽에 해당하는 핀들(TXD, RXD, +5V, +3.3V, GND와 CANH, CANL)을 배치하는 중이다. 연결 화살표를 묶음으로 처리하되 핀 순서를 맞춰서 시각적 대응을 명확히 하고, CAN_GND 연결을 강조하는 방식으로 정보 손실을 최소화하려고 생각 중.

이제 실제 좌표를 정해서 그리기로 결정했는데, Jetson 핀에서 ADM 로직 핀으로 가는 5개의 수평선을 그릴 때 양쪽 y 좌표를 일치시켜서 깔끔하게 연결하고, 버스 핀 쪽도 비슷하게 정렬할 계획.

ADM 박스 내부 격리벽과 핀 라벨 배치를 정확히 계산 중인데, 우측 끝이 412가 되도록 맞추고 있어.  좌핀과 우핀 모두 anchor start로 설정해서 좌핀 x=276, 우핀 x=348로 놓고, 박스 너비를 150으로 늘려서 우변이 418이 되게 조정했어. 이제 좌표를 확정하고 SVG 그리기 시작할 거야.

Writing SVG diagram... Adding the right-side pins and motor domain... The isolation barrier sits at x=340 between the left and right pins of the ADM section, rendered with a dashed red stroke to emphasize the domain separation. I'll add a descriptive label below explaining how it divides the two domains, then set the viewBox height to 320 to accommodate the motor ground label at y=192 and the barrier label at y=300.핵심은 — **로직 쪽(왼쪽)은 전부 Jetson 도메인 하나, 격리벽은 칩 안, 버스 쪽 GND만 모터에 붙는다**는 구조다. 그림으로 보면 명확하다.
```
This block is not supported on your current device yet.
```

격리벽의 좌측은 모두 Jetson으로 구성하고 우측은 GND와 모터만 배치하는 방식으로 정리하면 되겠다.왼쪽 5선은 전부 Jetson에서 따오면 되고(JP1만 3V3에 두고), 칩 안쪽 빨간 선이 격리벽이다. 그 너머 버스측 3선만 모터 쪽으로 가는데, 여기서 신경 쓸 건 주황색으로 표시한 CAN_GND → 배터리 negative 연결뿐이다. 이게 floating 안 되게 잡아주는 거다. termination 120Ω은 이 노드가 bus 끝이면 추가, 중간이면 생략.

### User · 2026-06-19T19:04:15.807932Z

DB9의 3번핀을 ODRIVE에 연결하면 되지? 아니면 배터리에 직접?

### User · 2026-06-19T20:01:41.447278Z

종단저항은 어떻게 연결?

### User · 2026-06-19T20:02:45.621245Z

내가 직접 케이블 만듦

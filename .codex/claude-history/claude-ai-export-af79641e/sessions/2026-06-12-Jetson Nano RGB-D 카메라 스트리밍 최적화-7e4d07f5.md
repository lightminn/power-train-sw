# Jetson Nano RGB-D 카메라 스트리밍 최적화

- Claude.ai conversation UUID: `7e4d07f5-08ec-4d7f-87b1-40ab79d610bb`
- Created: `2026-06-12T20:39:11.983083Z`
- Updated: `2026-06-25T20:32:42.006509Z`
- Categories: 파워트레인 관련 기록
- Source archive: `/home/light/Documents/카카오톡 받은 파일/data-af79641e-0ca7-4a6e-9a97-5140773682e0-1783641638-c1680b21-batch-0000.zip`

## Exported Summary

**Conversation overview**

The person is building a robot for two competitive events (극한로봇 and 국방로봇 대회), using a Jetson Orin Nano with a RealSense D435i RGB-D camera running YOLO-based object detection with XYZ coordinate overlays streamed to a laptop. The conversation covered the full technical stack from vision streaming to power electronics to display UI design. The person shared their GitHub repository (lightminn/power-train-sw) which Claude cloned and analyzed in detail throughout the conversation.

The conversation progressed through several technical domains. For vision streaming, Claude diagnosed the existing repository's core problems: the Orin Nano has no NVENC hardware encoder (the repo's comment misattributed this to a GStreamer ABI mismatch), the side-by-side color+depth stream wastes bandwidth because depth JET colormaps compress poorly under H.264, and YOLO results are burned into frames rather than sent as separate metadata. The recommended architecture is color-only x264enc zerolatency encoding via SRT for loss recovery in congested WiFi environments, with XYZ coordinates transmitted separately as JSON over UDP/WebSocket and overlaid on the laptop side. For networking, the person confirmed WiFi on the same LAN, and Claude recommended a dedicated travel router (GL.iNet Opal GL-SFT1200 as budget option, Beryl AX GL-MT3000 as preferred) mounted on the robot with the Jetson connected via Ethernet, avoiding soft AP mode entirely. The person confirmed 5GHz is non-negotiable for competition venue congestion avoidance.

For power electronics, the person shared a detailed topology: 12S LiPo (48V, 35Ah, 50.4V full charge) → PDIST80B PDB → three output branches driving ODrive v3.6 controllers, HAM series converters for AK45 and Dynamixel actuators, and a TDK iEA48004A180V isolated converter producing 18V for the Jetson. The router will be powered from the shared 18V rail via an 18V→5V/3A buck converter with an LC π filter and common-mode choke to isolate bidirectional noise. The person chose to hand-solder the filter on perfboard with epoxy potting for mechanical reliability rather than ordering a PCB. For the display, after ruling out DSI (Pi-specific, incompatible) and touch input (unwanted on the robot body), the person settled on a 7-inch DP display used purely as an output-only dashboard. Claude analyzed the existing motor_gui system (FastAPI backend, WebSocket telemetry at 100Hz, capabilities-based dynamic frontend) and proposed extending it with four data sources currently missing (vision, safety verdict, system health, link status) and adding a `/field` kiosk view showing: left panel with local D435i+YOLO preview independent of the SRT stream; top-right with US-100 safety verdict as a large SAFE/WARN/STOP indicator; mid-right with a 6-wheel rocker-bogie corner module health diagram; bottom-right with power rail voltage/current, tegrastats, and SRT link stats. The person prefers direct, technically precise communication and pushed back when recommendations were over-engineered (e.g., rejecting separate PCB for the filter, rejecting expensive Beryl AX in favor of Opal, rejecting a separate USB-C power tap from Jetson). Claude corrected course each time without requiring extended explanation.

## Conversation

### User · 2026-06-12T20:39:12.379961Z

nvidia jetson nano에 rgb-d 카메라 달고 yolo모델로 물체 xyz좌표 오버레이한 영상으로 내 노트북으로 쏴주고 싶은데 어떤 방식이 가장 품질이 좋고 저지연이야?

### Claude · 2026-06-12T20:40:34.783170Z

The user wants to stream YOLO object detection with XYZ coordinate overlays from a Jetson Nano with an RGB-D camera to a laptop, asking for the best quality and lowest latency approach. They're experienced with robotics and motion control systems, so they'll appreciate a technical solution.

I should note that the original Jetson Nano from 2019 is quite constrained for real-time YOLO with RGB-D processing—it might be worth clarifying if they mean the newer Orin Nano instead. For streaming, the key is leveraging Jetson Nano's hardware H.264/H.265 encoder (NVENC) through GStreamer to balance latency and quality, with options ranging from RTSP for compatibility to WebRTC or UDP for lower latency depending on their network setup. 메타데이터를 분리하는 게 핵심 설계 결정인데, 좌표를 영상에 직접 그리면 압축으로 텍스트가 손상되고 클라이언트에서 데이터를 활용할 수 없지만, 분리하면 영상 품질을 유지하면서 노트북에서 좌표를 로깅이나 다른 처리에 쓸 수 있다. 최적의 구성은 Jetson에서 YOLO와 깊이 정보로 검출한 좌표를 계산하고, 하드웨어 인코딩으로 영상을 압축한 후 WebRTC로 저지연 전송하는 것이다.

좌표는 WebRTC 데이터 채널로 따로 보내서 노트북에서 오버레이하면 되는데, 실제로는 로컬 네트워크 PoC라면 GStreamer의 UDP/RTP가 더 빠르게 구성할 수 있다. 절대 최저 지연이 필요하면 GStreamer와 하드웨어 인코딩으로 50-100ms 정도 달성 가능하고, 인터넷 통과와 안정성이 필요하면 WebRTC로 100-200ms 정도 감수해야 한다.

Jetson Nano의 NVENC는 H.265도 지원하지만 호환성과 디코딩 부담을 고려하면 H.264가 낫다. YOLO를 Nano에서 돌릴 때는 TensorRT로 최적화하고 nano 모델(YOLOv5n, YOLOv8n)에 INT8이나 FP16 양자화를 적용해야 프레임레이트가 나온다. RGB-D 카메라는 RealSense D435 같은 걸 기준으로 생각하고 있다.

답변 구조를 정리하면, 사용자가 간결함을 선호하고 평어체에 마크다운 헤더나 불릿을 최소화하길 원하지만 항목이 여러 개라 어느 정도 구조는 필요할 것 같다. 핵심은 Jetson Nano 오리지널에서 YOLO+RGB-D 실시간 처리가 빡빡하다는 전제를 명확히 하고, 좌표 burn-in vs 메타데이터 분리 같은 설계 결정을 제시하는 게 중요하다. 결론적으로 저지연과 품질을 모두 원하면 GStreamer 하드웨어 인코딩에 RTP/UDP가 로컬 환경에선 최선이고, WebRTC가 그 다음 선택지다. 구조를 잡으면 먼저 Nano 오리지널의 제약을 명확히 하고, 핵심은 좌표를 영상에 직접 그리지 말고 메타데이터로 분리해서 품질과 지연을 동시에 개선하는 것이다. 스트리밍은 로컬 최저지연이 필요하면 GStreamer의 NVENC 인코더와 RTP/UDP를, 안정성도 중요하면 WebRTC를 추천한다. 오리지널 Nano라면 YOLO와 depth를 동시에 처리하기가 정말 빠듯한데, 어쨌든 핵심 답은 두 경우 모두 같다. 저지연과 품질을 위해선 인코딩 방식과 전송 방법, 그리고 오버레이를 어디서 처리할지가 결정적이다.

오버레이 위치가 가장 중요한데, Jetson에서 좌표를 직접 프레임에 그려서 인코딩하면 간단하지만 텍스트가 H.264 압축에 뭉개지고 노트북에서 좌표 데이터를 재활용할 수 없다. 반대로 영상은 깨끗하게 인코딩하고 좌표는 별도 채널로 전송해서 노트북에서 오버레이하면 품질과 지연 모두 개선되고 좌표도 재활용할 수 있다.

인코딩은 Nano의 하드웨어 NVENC를 반드시 써야 한다. CPU 인코딩으로 x264를 쓰면 Nano의 CPU가 과부하되고 지연이 급증한다.

H.264가 H.265보다 노트북 디코딩 부담이 적고 호환성이 좋다. 전송 방식은 상황에 따라 다른데, 로컬 네트워크에서 최저지연이 필요하면 GStreamer RTP over UDP를 쓰면 50~100ms 수준이고 유선이면 패킷 손실도 문제 없다. 방화벽이나 무선 환경에서 안정성이 필요하면 WebRTC를 쓰되 100~200ms 정도 지연이 생기고 signaling 설정이 복잡하다. MJPEG나 RTSP는 구현이 쉽지만 지연과 품질 면에서 떨어진다.

결론적으로 최저지연을 원하면 GStreamer에 nvv4l2h264enc를 써서 RTP/UDP로 보내고, 좌표는 UDP나 WebSocket으로 분리하거나 안정성까지 필요하면 WebRTC의 data channel을 활용하는 게 좋다. 마지막으로 Nano 모델 사양도 확인해야 한다. 단순 gst-launch 한 줄로는 YOLO 처리를 포함할 수 없으니, Python에서 OpenCV와 appsrc를 통해 프레임을 파이프라인에 밀어넣는 구조가 필요하다. 다만 이건 복잡하니 개념만 설명하고 필요하면 구체적인 파이프라인을 짜주겠다고 하는 게 낫겠다. YOLO 결과인 바운딩박스와 좌표는 Jetson에서 계산한 후 JSON으로 UDP나 WebSocket으로 따로 송신하고, 영상 프레임은 그대로 인코딩해서 보내면 노트북에서 영상을 디코딩하면서 좌표 데이터를 받아 OpenCV로 오버레이하는 방식으로 타임스탬프로 동기화하면 된다.Jetson Nano가 오리지널(2019, Maxwell GPU)이면 먼저 짚고 갈 게 있다. YOLO + depth 처리를 실시간으로 돌리는 게 Nano에선 꽤 빡빡하다. TensorRT로 변환한 nano급 모델(YOLOv8n FP16/INT8)에 RealSense 같은 카메라 가정해도 10~25fps 선이다. 이게 전제로 깔려야 스트리밍 방식 논의가 의미가 있다. (Orin Nano면 얘기가 훨씬 여유롭다.)

본론. 품질·저지연의 핵심은 세 가지로 쪼개진다: 오버레이를 어디서 하느냐, 인코딩, 전송.

**오버레이 위치가 제일 중요하다.** XYZ 좌표를 Jetson에서 프레임에 직접 그려서(burn-in) 인코딩하면 구현은 간단한데, 텍스트가 H.264 압축에 뭉개지고 노트북에선 좌표를 숫자 데이터로 못 쓴다. 대신 영상은 깨끗하게 인코딩하고 좌표는 별도 채널로 JSON 보내서 노트북에서 그리는 게 낫다. 품질·지연 둘 다 이득이고 좌표 재활용(로깅, 후처리)도 된다. 대신 프레임-좌표 sync를 위해 frame id나 timestamp를 좌표에 붙여야 한다.

**인코딩은 하드웨어 NVENC 무조건.** GStreamer `nvv4l2h264enc`. CPU 인코딩(x264) 쓰면 Nano CPU가 죽고 지연이 폭증한다. H.265도 되지만 H.264가 노트북 디코딩 부담 적고 호환성 좋다.

**전송 방식:**

- GStreamer RTP over UDP — 같은 LAN, 특히 유선이면 최저지연. 50~100ms대. 패킷 손실에 약하지만 유선이면 거의 문제 없다.
- WebRTC — 무선/방화벽/안정성이 필요할 때. 100~200ms, jitter buffer로 안정적. 대신 signaling 서버 등 셋업이 복잡하다.
- RTSP / MJPEG — 띄우긴 쉬운데 지연·품질 양쪽에서 위 둘에 밀린다.

결론: **로컬에서 최저지연이 목표면 GStreamer + `nvv4l2h264enc` + RTP/UDP**, 좌표는 별도 UDP나 WebSocket. 무선이거나 안정성까지 원하면 WebRTC(영상) + data channel(좌표) 조합.

YOLO 결과를 파이프라인에 끼워넣어야 하니 보통 `gst-launch` 한 줄로는 안 되고 Python에서 appsrc로 프레임을 밀어넣는 구조가 된다.

카메라는 RealSense D435 계열로 보면 되나? 그리고 Nano가 오리지널인지 Orin인지만 확정해주면 파이프라인 구체적으로 짜줄게.

### User · 2026-06-12T20:45:08.851584Z

같은 LAN, wifi연결 예정, Jetson Orin Nano임. 카메라는 D435i. 극한/국방로봇 대회장에서 사용할 예정. 자세한 대회 규칙은 프로젝트 파일 참고. https://github.com/lightminn/power-train-sw 여기 vision 및 scripts 폴더에 테스트할때 썼던 스크립트 있는데, 품질이 만족스럽지는 않았었음.

### Claude · 2026-06-12T20:48:59.105681Z

사용자가 추가 정보를 제공했는데, 같은 LAN과 WiFi로 연결할 Jetson Orin Nano(원본 Nano보다 훨씬 성능이 좋음)와 D435i 카메라를 사용하며, 극한/국방로봇 대회에서 이 시스템을 운영할 예정이다. 대회 규칙은 이미 제공된 PDF 파일들에 있고, GitHub 저장소도 공유받았다.

GitHub 저장소의 vision과 scripts 폴더에 있는 테스트 스크립트들이 품질 기준을 충족하지 못하고 있다. 저장소를 직접 클론해서 해당 폴더들의 구조와 스크립트 내용을 확인해야 한다. 동시에 이미 컨텍스트에 있는 극한로봇과 국방로봇 대회 규칙 PDF를 검토해서 vision 및 스트리밍과 관련된 요구사항이 무엇인지 파악해야 한다.

극한로봇 대회에서는 원격주행 시 조종자가 로봇의 비전시스템을 통해서만 경기장을 볼 수 있고, 자율주행 중에는 복제된 로봇 화면을 보며 화재 규모, 구조요구자, 피아식별 같은 vision 태스크를 수행해야 한다. 국방로봇 대회도 유사하게 운용자가 조종스테이션에서 로봇의 카메라와 센서 정보로만 상황을 판단하며, 조종화면이 외부로 미러링되는 구조다.

이 vision 스트리밍은 대회의 핵심인데, 원격주행 시 조종자가 카메라 영상만으로 조종해야 하므로 저지연이 매우 중요하다. XYZ 좌표 오버레이는 물체 식별과 거리 판단에 도움이 되고, 피아식별, 물체 인식, 신호등 인식, 선도로봇 추종 같은 다양한 vision 태스크들이 있다. 다만 대회장에서 여러 팀이 동시에 운영하면서 WiFi 간섭과 혼잡이 심할 것으로 예상된다.

2.4GHz는 특히 혼잡하니 5GHz를 써야 하고 채널 선택이 중요한데, 국방로봇은 동시다발적으로 진행되고 극한로봇은 연막 속에서 시야가 안 좋아서 영상 품질이 더욱 중요하다. 결국 WiFi 저지연 스트리밍을 대회장에서 안정적으로 하려면 혼잡 대응이 핵심이고, 유선이면 UDP RTP로 최저지연을 달성할 수 있다.

그런데 WiFi가 확정되고 대회장 혼잡 환경이 현실이니 전략을 다시 생각해야 한다. 순수 UDP는 패킷 손실에 약해서 WiFi 혼잡 상황에선 영상이 깨질 수 있는데, WebRTC는 혼잡 제어와 적응형 비트레이트로 이런 환경에 훨씬 강하다. 저지연만 보면 RTP/UDP가 빠르지만, 대회장 환경에선 손실 복원력이 더 중요할 수 있으니 이 트레이드오프를 명확히 해야 한다.

WebRTC는 Google의 혼잡 제어와 FEC, NACK 같은 손실 복원 메커니즘이 있고 Orin Nano에서도 충분히 돌아가지만 셋업이 복잡하다. 반면 SRT는 저지연과 손실 복원(ARQ 기반)을 모두 제공하면서 UDP 기반이라 WiFi 환경에 적합하고, GStreamer에서도 지원되며 셋업이 더 간단해서 대회장 환경에선 좋은 절충안이 될 수 있다.

코드를 직접 봐야 품질 문제의 원인을 파악할 수 있으니 먼저 저장소를 클론해서 인코딩 방식, 전송 프로토콜, depth-RGB 정렬 방식, YOLO 실행 방식 등을 확인한 후 구체적인 개선안을 제시하겠다. D435i는 RGB와 depth 정렬이 필수인데 rs.align을 사용해야 한다.

depth 값을 3D 좌표로 변환할 때는 카메라 내부 파라미터를 이용해 픽셀을 역투영하고, bbox 중심의 depth 값을 가져오면 된다. 단일 픽셀보다는 ROI 중앙값을 사용하는 게 노이즈에 더 강하다. 품질 불만의 원인은 소프트웨어 인코딩 품질 저하, 해상도/비트레이트 설정 문제, WiFi에서 손실에 약한 전송 프로토콜, 또는 YOLO와 depth 처리 과정의 문제일 가능성이 높다.

CPU에서만 모든 처리를 하면 병목이 생겨 프레임이 떨어질 수 있고, 좌표를 이미지에 그려 넣으면 압축 손실이 발생할 수 있다. 실제 원인을 파악하려면 코드를 직접 봐야 하는데, 먼저 저장소를 클론한 후 vision과 scripts 폴더를 살펴보자. 그 다음 Orin Nano와 D435i, WiFi 환경에 맞는 아키텍처를 제안하고, 대회장 WiFi 혼잡이라는 핵심 리스크를 고려해 WebRTC나 SRT 같은 전송 프로토콜을 선택하는 방향으로 구체적인 개선안을 제시할 계획이다.

코드 진단을 먼저 하고 사용자가 원하는 간결한 답변을 제공하기 위해 저장소를 클론해서 살펴보자. GitHub 도메인이 허용되어 있으니 클론이 가능하고, 작업 디렉토리는 /home/claude에서 진행하겠다.
```
This block is not supported on your current device yet.
```

 vision 폴더가 motor_control 아래에 있고 scripts 폴더도 있으니 둘 다 확인해봐야겠다.
```
This block is not supported on your current device yet.
```

 핵심 파일들을 살펴봐야 할 것 같다. realsense_stream.py는 스트리밍을 담당하고, yolo_cuda_stream.py가 YOLO와 스트리밍을 함께 처리하는 핵심 부분인 것 같다. 그 외에 yolo_depth_3d.py는 YOLO와 깊이 정보를 3D 좌표로 변환하고, 수신 쪽에는 recv_stream.sh와 laptop_client_video.py가 있다. 이 파일들을 분석해야 문제를 진단할 수 있을 것 같은데, 특히 yolo_cuda_stream.py가 핵심인 것 같다.
```
This block is not supported on your current device yet.
```

 발견한 핵심 문제는 소프트웨어 기반의 `openh264enc` 인코더를 사용 중이라는 점인데, 주석에 NVENC를 사용할 수 없어서 이 방식으로 대체했다고 명시되어 있다. 이게 화질 저하의 주요 원인일 가능성이 높다. 흥미롭게도 Jetson Orin Nano는 실제로 NVENC 하드웨어 인코더를 지원하지 않기 때문에, 소프트웨어 인코딩이 현재 상황에서는 피할 수 없는 선택이다. 다른 문제들도 보인다. side-by-side로 color와 depth를 나란히 붙여서 1280x480으로 보내면 해상도와 대역폭이 2배가 되는데, depth 컬러맵은 압축하기 어려운 고주파 노이즈 데이터라서 4Mbps 비트레이트가 부족할 수 있고 color 품질까지 떨어뜨린다. 그리고 raw BGR을 stdin으로 gst에 파이프하는 방식도 있는데...

매 프레임마다 tobytes() 변환으로 1.84MB를 55MB/s 속도로 파이프에 쓰는 건 CPU 부담이 크고, videoconvert로 BGR을 I420으로 소프트웨어 변환하는 것도 CPU를 많이 쓴다. 좌표나 텍스트를 cv2.putText로 프레임에 직접 그리면 압축 손실이 생기고 앞의 문제들이 더 악화된다. 결국 소프트웨어 인코딩과 side-by-side 구성, depth colormap의 압축 비효율이 핵심 문제인데, Orin Nano에 NVENC가 없다면 앞 답변의 하드웨어 인코딩 조언이 맞지 않을 수 있다.

Orin Nano에서 저지연 고품질 스트리밍을 하려면 소프트웨어 인코더를 최적화하거나, depth를 따로 처리해서 color만 인코딩하고 depth는 좌표 메타데이터로만 전송하는 방식을 고려할 수 있다. 또는 Orin Nano에 NVJPEG 하드웨어 엔진이 있는지 확인해볼 필요가 있다. MJPEG를 하드웨어로 인코딩할 수 있다면 H.264보다 대역폭은 더 필요하지만 지연이 적고 CPU 부하가 없어서 WiFi 대역폭이 충분하면 좋은 옵션이 될 수 있다. Orin Nano의 인코딩 능력을 확인하기 위해 검색이 필요하고, 그 다음 yolo_cuda_stream.py와 yolo_depth_3d.py 파일들을 살펴봐서 좌표 계산과 YOLO 처리 방식을 진단해야 한다.
```
This block is not supported on your current device yet.
```

 이제 yolo_depth_3d.py와 노트북 클라이언트를 살펴보고 있는데, 핵심 문제들이 드러나고 있다. NVENC를 사용할 수 없는 이유가 두 가지인데, 하나는 Orin Nano의 NVENC 자체 문제이고 다른 하나는 L4T 플러그인 ABI 버전과 컨테이너의 GStreamer 버전 간 불일치 때문이다.

실제로 Orin Nano는 NVENC 하드웨어가 없어서 nvv4l2h264enc를 사용할 수 없을 가능성이 높은데, 개발자가 ABI 불일치로 진단한 것 같다. 대신 소프트웨어 인코딩인 openh264enc를 사용하고 있으며 4Mbps 비트레이트로 설정되어 있다.

YOLO 결과의 bbox와 라벨을 프레임에 직접 그려서 압축하고, 원본 BGR 데이터를 subprocess stdin으로 파이프하면서 매 프레임마다 tobytes() 변환과 소프트웨어 videoconvert를 거친다. USB 카메라는 cv2.VideoCapture로 처리하고 있으며, yolo_cuda_stream은 일반 USB 카메라를 MJPG로 사용하고 realsense_stream은 RealSense SDK를 별도로 사용하는 구조다.

사용자가 D435i와 YOLO를 함께 사용하면서 3D 좌표를 통합해야 하는데, dustynv 컨테이너의 GStreamer ABI 불일치가 NVENC 문제의 근본 원인으로 보인다. Jetson Orin Nano의 실제 인코딩 능력을 확인하고 yolo_depth_3d.py의 좌표 계산 방식을 살펴봐야 한다.
```
This block is not supported on your current device yet.
```

 이제 상황이 명확해졌다. 핵심 문제는 소프트웨어 인코딩인 openh264enc를 사용하고 있다는 것인데, 실제로는 NVENC를 활용해야 한다. 원인은 L4T 플러그인 ABI 버전 불일치로 보인다 - yolo_cuda_stream.py의 주석에서 NVENC(nvv4l2h264enc)가 L4T 플러그인 ABI 1.14와 컨테이너의 GStreamer 1.20+ 버전 간의 호환성 문제를 언급하고 있다.

하지만 더 근본적인 문제가 있을 수 있다. Orin Nano 자체에 NVENC 하드웨어가 없을 가능성이 있다. 이를 확인해야 한다. 만약 정말 하드웨어가 없다면, ABI 불일치는 부차적인 문제이고, 개발자의 진단이 잘못된 것이다.

추가로 side-by-side 컬러와 깊이 이미지를 1280x480으로 합쳐 보내는데, depth colormap(JET)의 급격한 색 변화가 H.264 압축 효율을 떨어뜨려 컬러 품질을 손상시키고 있다. 그리고 YOLO 결과를 프레임에 직접 그려 넣으면서 압축 손실이 증가하고, 노트북에서 좌표 데이터를 활용할 수 없게 되었다. laptop_client_video.py는 완전히 다른 경로를 사용하고 있다.

TCP JPEG 디코딩과 UDP RTP H.264 수신 경로가 불일치하고 있으며, TCP는 저지연 스트리밍에 부적합하다. WiFi 환경에서 순수 UDP RTP를 사용하면서 손실 복구 메커니즘이 없어서 혼잡한 환경에서 화면 깨짐이 발생한다. 컨테이너 내 GStreamer와 OpenCV 간의 호환성 문제도 있다.

이제 Orin Nano의 인코딩 능력을 확인해야 한다. 이 정보가 전체 아키텍처 방향을 결정한다. 검색 후 근본 원인들을 진단하고, Orin Nano의 NVENC 지원 여부에 따라 전략을 재설정한 다음, depth를 별도 메타데이터로 분리하고 color만 효율적으로 인코딩하는 구조를 제안할 계획이다. 소프트웨어 인코딩만 가능한지, 아니면 JPEG 하드웨어 인코더가 있는지 Orin Nano 스펙을 확인해야 할 것 같다.
```
This block is not supported on your current device yet.
```

 확정됐다. Jetson Orin Nano는 NVENC 하드웨어 인코더가 없어서 GPU 인코딩이 불가능하고 소프트웨어 인코딩만 가능하다는 게 공식 문서와 포럼에서 일치한다. 그래서 개발자가 언급한 "L4T plugin ABI 불일치" 문제는 다른 원인에서 비롯된 것 같다.

이제 NVENC 없이 저지연 고품질 스트리밍을 구현하려면 x264enc 소프트웨어 인코딩을 zerolatency 튜닝으로 최적화하거나, 컬러 영상만 전송하고 depth는 좌표 데이터로 대체하거나, 해상도를 낮춰서 CPU 부하를 줄이는 방식을 고려해야 한다.

전송 프로토콜도 WiFi 혼잡 환경에서 손실 복구가 필요한데, WebRTC는 견고하지만 셋업이 복잡하고, SRT는 UDP 기반 ARQ로 저지연과 손실복구를 절충할 수 있으며, RTP에 FEC를 추가하는 방식도 있다. 대회 규정상 조종자가 경기장을 직접 볼 수 없다는 제약이 있으니 이를 고려해서 선택해야 한다. 로봇과 노트북을 직접 연결하거나 전용 라우터를 써서 대회장 공용 WiFi에 의존하지 않아야 하고, 혼잡 상황에 대비해 비트레이트를 보수적으로 설정해야 한다. 색상 영상은 별도로 인코딩해서 보내고, YOLO 감지 결과와 좌표는 JSON으로 작은 UDP/WebSocket 패킷으로 분리 전송해서 노트북에서 디코딩한 영상 위에 OpenCV로 오버레이하는 방식으로 영상 품질을 유지하면서 좌표 정보를 효율적으로 전달할 수 있다.

D435i 카메라는 이미 depth-color 정렬과 robust median 필터링을 적용하고 있으며, XYZ 좌표 계산도 기존 스크립트에서 처리하고 있는데 정확한 deproject 함수 사용 여부는 확인이 필요하다.

이제 진단과 권장사항을 정리해서 답변하자. 핵심은 Orin Nano에 NVENC 하드웨어가 없다는 점인데, 저장소의 "ABI 불일치" 주석은 잘못된 진단이고 실제로는 소프트웨어 인코딩만 가능하다. 그 위에서 품질을 떨어뜨리는 요소들—color와 depth를 나란히 배치하는 대역폭 낭비, depth colormap의 압축 효율 문제, openh264enc 대신 x264enc의 zerolatency 튜닝 활용—을 개선해야 한다.

전송 단계에서는 대회장의 WiFi 혼잡 환경에서 손실 복구 없는 순수 UDP RTP가 끊김의 원인이므로 SRT나 WebRTC로 전환하고, 구조적으로는 color만 인코딩하고 좌표는 별도로 송신하는 방식을 고려해야 한다. 원격주행 규정상 영상 안정성이 최우선이므로 5GHz 전용 대역과 직결 연결, 연막 구간 대비가 필수다. x264enc의 zerolatency 튜닝이 Orin Nano에서 최선이고, 컬러 영상만 보내면서 좌표는 별도 메타데이터로 분리하는 게 핵심이다. SRT로 전송하면 WiFi 환경에서 저지연과 손실 복구를 동시에 얻을 수 있다. 송신 파이프라인 예시를 간단히 보여주고 마무리하자.

가장 중요한 건 NVENC가 없다는 점이다.repo 봤다. 품질 불만의 근본 원인이 코드 주석에 잘못 적혀 있다. 거기부터 짚고 간다.

**Orin Nano엔 NVENC가 아예 없다.** `yolo_cuda_stream.py` 주석엔 "L4T plugin ABI 1.14 vs 컨테이너 GStreamer 1.20+ 불일치로 사용 불가"라고 돼 있는데, 그게 원인이 아니다. Orin Nano는 NVENC 엔진 자체가 없다. nvv4l2h264enc는 NVENC 하드웨어를 쓰는 플러그인이라 Orin Nano에선 동작하지 않고, GPU로 인코딩하는 플러그인도 없어서 소프트웨어 인코더가 유일한 방법이다. 하드웨어 인코딩이 필요하면 Orin NX로 가야 한다. 컨테이너 밖에서, docker-compose에서 `.so` 수동 마운트해도 안 된다. 하드웨어가 없으니까. 그러니 SW 인코딩을 전제로 최적화하는 게 맞는 방향이다 — 다만 그걸 잘 못 하고 있다.

품질을 깎아먹는 것들, 영향 큰 순서로:

1. **depth를 영상으로 보내고 있다.** `realsense_stream.py`의 side-by-side는 1280×480이라 대역폭이 2배인 데다, depth JET 컬러맵은 인접 픽셀 색이 급변(고주파)해서 H.264가 압축하기 최악이다. 한정된 SW 인코딩 비트레이트를 depth 노이즈가 잡아먹어서 color 화질이 떨어진다. **color만 보내고 XYZ 좌표는 메타데이터로 분리해라.** 너가 처음에 말한 "오버레이 영상"은 좌표를 프레임에 굽지 말고, color 영상 따로 + 좌표 JSON 따로 보내서 노트북에서 그리는 구조가 정답이다. `results[0].plot()` burn-in도 같은 이유로 빼라 — 압축 손실 나고, 노트북에서 좌표를 데이터로 못 쓴다. frame id만 좌표에 붙이면 sync 맞춰서 OpenCV로 그릴 수 있다. 좌표 패킷은 작아서 영상이 잠깐 깨져도 살아있는 것도 이점.

2. **인코더 선택.** `openh264enc`보다 `x264enc tune=zerolatency speed-preset=superfast`가 같은 비트레이트에서 화질·지연 둘 다 낫고 튜닝 폭이 넓다. color 단독이면 인코딩 부하도 절반이라 6코어 A78이 YOLO랑 나눠 쓸 여유가 생긴다.

3. **전송이 WiFi 대회장엔 안 맞다.** 순수 UDP RTP는 손실 복구가 없다. `rtpjitterbuffer`는 순서·지터만 흡수하지 패킷 손실은 못 메운다. 대회장은 여러 팀이 동시에 WiFi를 쓰는 극혼잡 환경이라 블록 노이즈·프리징이 난다. **SRT를 써라** — UDP 기반이라 저지연인데 ARQ로 손실을 재전송 복구한다(`latency` 파라미터로 지연 예산 조절). GStreamer `srtsink`/`srtsrc`로 비교적 쉽게 붙는다. 더 견고한 건 WebRTC(congestion control+NACK+FEC)지만 signaling 등 셋업이 무겁다. 우선순위는 SRT부터.

대회 규정 맥락이 이 설계를 더 강하게 누른다. 두 대회 다 원격주행 시 조종자는 경기장을 직접 못 보고 로봇 카메라 영상으로만 조종해야 한다(극한 6조, 국방 운영방침 3항). 즉 영상 스트림이 끊기면 그 구간은 그냥 실패다. **저지연보다 안정성이 먼저다.** 추가로:

- WiFi는 대회장 공용 AP에 절대 의존하지 마라. 5GHz 전용으로, 가능하면 로봇↔노트북 전용 라우터나 노트북 AP 모드로 직결해라. 국방대회는 "동시다발 진행"이라 2.4GHz는 사실상 죽는다.
- 극한대회 화재진압 구간은 연막(유독가스)이라 color 영상 자체가 안 보인다. 이 구간만큼은 depth나 좌표 데이터가 주 정보원이 될 수 있으니, 좌표 분리 채널이 여기서 진짜 값을 한다.

권장 정리: **color만 x264enc zerolatency로 SRT 송신 + 좌표는 별도 UDP/WS JSON, 노트북에서 합성.**

송신 인코딩 부분만 예시로:

```
... ! videoconvert ! video/x-raw,format=I420
  ! x264enc tune=zerolatency speed-preset=superfast bitrate=3000 key-int-max=30
  ! h264parse ! mpegtsmux ! srtsink uri="srt://:5000?mode=listener&latency=120"
```

`laptop_client_video.py`는 `tcpclientsrc ! jpegdec`로 돼 있어서 `recv_stream.sh`(UDP RTP H.264)랑 경로 자체가 안 맞는다 — 옛 Pi MJPEG 코드 잔재로 보인다. 수신측도 SRT로 통일해야 한다.

좌표 분리 송수신 코드랑 노트북 합성/sync 부분까지 짜줄까? 아니면 SRT 셋업부터 검증하고 싶나 — 하나만 골라줘.

### User · 2026-06-14T09:21:24.273380Z

그러면 대회장에서 wifi AP를 뭘 써야할까? 대회장 공용 와파는 사실상 못쓸수준일테고 젯슨이나 노트북중 하나가 AP가 되어야할텐데 출력이 부족하거나 하지 않을까

### Claude · 2026-06-14T09:23:32.880030Z

사용자가 대회장 환경에서 WiFi AP로 Jetson이나 노트북 중 어느 것을 써야 할지 고민하고 있는데, 출력 성능이 충분할지 걱정하고 있다. RF 혼잡한 환경에서 로봇과 노트북 간의 무선 연결을 안정적으로 구성하는 게 핵심 문제인 것 같다.

사실 Jetson이나 노트북을 soft AP로 쓰는 것보다는 별도의 전용 라우터를 가져가는 게 출력과 안정성 면에서 훨씬 낫다. 로봇과 노트북 둘 다 클라이언트로 연결하고 라우터가 AP 역할을 하는 구성이 일반적이다. 대회장 조종스테이션에 전원이 있다면 AC 전원 라우터를 설치할 수 있을 것 같다. 극한로봇이나 국방로봇 대회 구조를 생각해보면 조종스테이션과 경기장 거리가 그리 멀지 않을 테니 이런 구성이 실용적일 것 같다.

실제 거리는 실내 수십 미터 수준이라 WiFi 커버리지는 충분하지만, 진짜 문제는 거리가 아니라 다른 팀들의 신호 간섭이다. Jetson Orin Nano의 내장 WiFi 모듈이 soft AP 모드를 제대로 지원하는지부터 확인해야 한다.

Soft AP 방식은 여러 문제가 있다. Jetson의 WiFi 카드든 노트북의 핫스팟이든 출력이 약하고, 특히 5GHz에서는 DFS 채널 제약이 있으며, 동시에 인터넷 연결을 유지하기 어렵다. 안테나도 작아서 혼잡한 환경에서 신호가 약해진다.

결론적으로 전용 라우터나 AP를 따로 가져가는 게 낫다. Jetson과 노트북을 그냥 클라이언트로 연결하면 라우터의 강한 출력과 제대로 된 안테나 덕분에 훨씬 안정적이다.

라우터 선택할 때는 5GHz 지원이 필수고 (2.4GHz는 대회장에서 간섭이 심함), WiFi 5 이상이 좋다. 채널을 수동으로 고정할 수 있어야 하고, 작고 휴대 가능한 여행용 라우터가 이상적이다. GL.iNet 같은 OpenWrt 기반 라우터들이 좋은 선택지다.

다만 대회장에서 여러 팀이 5GHz를 쓰면 혼잡해질 수 있으니, 더 견고한 방법을 생각해봐야 한다. 유선 연결이 가장 안정적이지만 로봇이 움직이는 환경에서는 케이블이 걸리는 문제가 있다. 다른 주파수 대역을 활용하는 방법도 있는데, 일반 라우터는 5GHz UNII 대역을 쓰고 DFS 채널 같은 옵션도 있다.

결국 WiFi 6를 지원하는 전용 travel router를 쓰면서 5GHz 채널을 수동으로 고정하는 게 실전에서 가장 현실적인 해결책이다.

이제 라우터를 어디에 배치할지가 핵심인데, 조종스테이션 쪽에 고정으로 두고 로봇과 노트북이 모두 클라이언트로 연결하는 방식과 로봇 자체에 라우터를 탑재해서 로봇이 AP 역할을 하는 방식 사이에서 선택해야 한다.

옵션 B가 더 유리해 보인다. 작은 travel router를 로봇에 USB로 구동하고 Jetson과 라우터를 이더넷으로 직결하면, Jetson의 불안정한 내장 WiFi soft AP를 피할 수 있고 라우터의 강한 신호와 안테나를 활용할 수 있다. 노트북은 단순히 5GHz 클라이언트로 연결되면 된다. 다만 로봇에 라우터의 무게와 전원이 추가되는 단점이 있지만, travel router는 충분히 작고 USB-C 전원으로 구동 가능하다. 외장 안테나를 로봇 위쪽에 노출시켜서 금속 프레임의 차폐를 피해야 한다. 대회 당일에 WiFi 스캐너로 가장 빈 5GHz 채널을 찾아 수동으로 고정하고, 혼잡한 환경에서는 채널폭을 20MHz로 좁혀서 간섭에 더 강하게 설정하는 게 낫다.

결론적으로 Jetson이나 노트북의 soft AP는 출력과 안테나 성능이 약해서 전용 travel router를 로봇에 탑재하고 Jetson을 이더넷으로 직결한 후 라우터를 AP로 운영하는 게 가장 견고한 구성이다.

라우터는 5GHz WiFi 6 지원, 채널 수동 고정, USB-C 전원 가능한 소형 제품(GL.iNet 같은)을 찾아야 하고, 현장에서 빈 채널을 스캔해 고정한 뒤 20/40MHz 좁은 대역폭으로 설정하고 안테나를 로봇 위로 노출시키면 된다.

구체적인 제품 추천을 위해서는 최신 정보 검색이 필요할 것 같다. 검색으로 현재 GL.iNet 라인업을 확인하고 구체적인 모델 한두 개를 제시하되, 핵심은 원리 설명에 두자. 사실 검색 없이도 soft AP 피하고 전용 라우터 사용, 로봇에 직결하는 원칙만 설명해도 충분할 수 있다.
```
This block is not supported on your current device yet.
```

사용자의 우려가 타당하다는 걸 확인했다. Jetson이나 노트북을 soft AP로 사용하면 출력이 약해서 문제가 될 수 있으니, 전용 라우터를 쓰는 게 더 나은 구성이다.

최적의 방법은 로봇에 작은 travel router를 탑재하고 Jetson을 유선으로 연결하는 것이다. 그러면 라우터가 AP 역할을 하고 노트북은 클라이언트가 되는데, 이렇게 하면 무선 구간이 라우터와 노트북 사이 하나로 줄어들고 제대로 된 안테나와 전력으로 안정적인 연결을 유지할 수 있다.

라우터는 5GHz, WiFi 6 이상, USB-C 전원 지원이 필수인데, GL.iNet Beryl AX 같은 소형 라우터가 이 조건을 만족한다. DFS 지원이 있으면 혼잡한 주파수를 피할 수 있어서 더 좋다.

현장에서는 WiFi 스캐너로 빈 5GHz 채널을 찾아 수동으로 고정하고, DFS 채널이 비어있으면 활용해서 다른 팀의 간섭을 최소화한다.  채널폭을 20/40MHz로 좁게 설정하면 영상 전송에 충분하면서 간섭에 더 강해지고, 라우터 안테나를 로봇 금속 바디 위로 노출시켜 신호를 최적화한다. 로봇의 48V 배터리에서 DC-DC 컨버터로 5V USB를 만들어 라우터에 공급하면 전력 부담이 거의 없다.

실내 수십 미터 거리에서는 거리보다 간섭이 문제인데, 험지나 금속 구조물 뒤로 로봇이 가면 신호가 약해질 수 있다. 라우터를 로봇에 실으면 로봇과 노트북 사이의 직선 경로가 중요하고, 조종 스테이션의 안테나 위치도 신경 써야 한다. 라우터를 고정점에 두는 방법도 있지만 로봇이 금속 장애물 사이를 움직이면서 신호가 약해질 위험이 있다.

로봇 탑재가 더 안정적인데, 라우터가 로봇과 함께 이동하고 노트북은 한 곳에 고정되기 때문이다. 다만 라우터와 노트북 사이는 여전히 무선이고 로봇이 멀어지거나 장애물 뒤로 가면 끊길 수 있으니, 결국 무선 홉 하나는 피할 수 없다. 핵심은 그 홉을 Jetson 내장 칩이 아니라 제대로 된 라우터가 담당하고 채널을 잘 선택하는 것이다.

해결책은 전용 travel router를 로봇에 탑재하고 Jetson을 유선으로 직결하는 것이다. GL.iNet Beryl AX 같은 모델이 USB-C 전원과 DFS를 지원해서 좋다. 현장에서는 채널을 수동으로 고정하고 DFS를 활용하며 좁은 채널폭을 사용하고 안테나를 노출시켜야 한다. 전원은 48V에서 5V USB를 따서 구동하면 되니 부담 없다.soft AP는 피하는 게 맞다. 너 우려가 정확하다. Jetson 내장 WiFi나 노트북 핫스팟을 AP로 쓰면 출력·안테나가 빈약하고, soft AP 드라이버 자체가 불안정한 경우가 많다(특히 5GHz). 대회장 혼잡까지 겹치면 답이 없다.

권장 구성은 **전용 travel router를 로봇에 싣고, Jetson은 이더넷으로 라우터에 직결, 라우터를 AP로, 노트북이 client로 붙는 것**이다. 핵심은 이 배치의 효과다 — 무선 구간이 라우터↔노트북 한 hop으로 줄고, 그 hop을 Jetson 내장칩이 아니라 제대로 된 PA·안테나를 가진 라우터가 담당한다. Jetson↔라우터는 유선이라 거기서 무선 손실이 안 난다. soft AP 불안정성을 통째로 우회하는 거다.

라우터 조건:
- 5GHz, WiFi 6 이상
- **USB-C 전원** — 로봇 48V에서 DC-DC로 5V만 따면 구동된다. 무게·전력 부담 거의 없음
- 채널 수동 고정 가능
- DFS 지원이면 혼잡 회피에 큰 도움

GL.iNet이 이 용도(OpenWrt 기반이라 채널·출력 제어 자유)에 사실상 표준이다. Beryl AX(GL-MT3000)가 무난한데, DFS를 지원해서 5GHz에서 평소 군·위성·기상 레이더용으로 예약된 추가 주파수를 쓸 수 있고, 이 추가 채널이 고밀도 환경의 혼잡을 낮춘다. 대회장처럼 모두가 일반 5GHz 채널에 몰릴 때 DFS 대역이 비어있을 확률이 높다는 게 포인트다. 신형으로는 Beryl 7(GL-MT3600BE)이 USB-C로 표준 폰 충전기 케이블로 구동되고 WiFi 7을 지원한다.

현장 튜닝이 라우터 선택만큼 중요하다:

- 대회 당일 노트북 WiFi 스캐너로 가장 빈 5GHz 채널을 찾아 **수동 고정**해라. 자동 채널 선택은 끈다.
- 비어있으면 DFS 채널을 노려라. 단 DFS는 레이더 감지 시 채널을 바꾸느라 수십 초 끊기는 리스크가 있으니 리허설 때 그 채널로 미리 돌려서 안정성 확인해라.
- 채널폭은 20~40MHz로 좁게. 영상은 3~5Mbps면 충분하니 대역폭은 남아돌고, 좁을수록 간섭에 강하다.
- 라우터 외장 안테나를 로봇 금속 프레임 위로 노출시켜라. 바디 안에 묻으면 차폐된다.

한 가지 짚자면, 라우터를 로봇에 싣든 조종스테이션 옆 고정점에 두든 라우터↔노트북(또는 로봇) 한 hop은 무선이라 험지·장애물 구간에서 로봇이 금속 구조물 뒤로 가면 약해지는 건 불가피하다. 그래서 인코딩·전송을 SRT 같은 손실 복구 방식으로 깐 게 의미가 있는 거다 — 링크가 잠깐 나빠져도 복구되니까.

DFS 채널 전략까지 리허설 체크리스트로 정리해줄까?

### User · 2026-06-14T09:28:05.472478Z

12S LiPo (48V, 35Ah, 완충 50.4V)
        │
        └─ PDIST80B (48V 메인 PDB, Max 80A, 출력 단자 3개)
             │
             ├─ 출력 A (peak)
             │    ├─ ODrive v3.6 #1 → BL70200 ×2
             │    ├─ HAM6227 #1 → 24V → AK45-36 ×2
             │    └─ HAM5230a → 12V → Dynamixel ×6
             │
             ├─ 출력 B (peak)
             │    ├─ ODrive v3.6 #2 → BL70200 ×2
             │    └─ HAM6227 #2 → 24V → AK45-36 ×2
             │
             └─ 출력 C (peak)
                  ├─ ODrive v3.6 #3 → BL70200 ×2
                  └─ TDK IEA48004A180V (절연) → 18V → Jetson

전체 파워 토폴로지가 이런데, 어떤 travel router를 사는게 좋지? 전원연결 어떻게 하지?

### Claude · 2026-06-14T09:30:35.393385Z

사용자가 전체 파워 토폴로지를 제시했고, 이제 travel router 선택과 전원 연결 방법을 묻고 있다. 토폴로지를 분석해보니 12S LiPo 배터리(48V nominal, 35Ah, 완충 시 50.4V)에서 PDIST80B 메인 PDB(80A max)를 통해 3개의 출력으로 ODrive와 BLDC 모터들을 구동하는 구조인데, 설명이 끝나지 않았다.

라우터 전원 공급이 핵심 문제다. Travel router는 보통 USB-C 5V/3A(15W) 정도를 요구하는데, 몇 가지 옵션이 있다. Jetson의 USB 포트에서 직접 따는 건 전류 제한과 Jetson 자체의 전력 부족 때문에 비추천이고, 대신 48V에서 5V로 변환하는 별도 DC-DC 벅 컨버터를 추가하는 게 가장 깔끔한 방법이다. 어느 레일에서 전원을 끌어올지가 노이즈와 안정성에 영향을 미치는데, 사용자가 EMI에 신경을 많이 쓰는 것 같다.

라우터는 통신 장비라 깨끗한 전원이 중요하고, 모터 드라이버와 같은 레일에서 전원을 끌면 스위칭 노이즈와 회생제동 전압 스파이크 위험이 있다. Jetson이 TDK 절연 컨버터로 보호받고 있는 것처럼, 라우터도 비슷한 방식으로 보호하는 게 좋겠다.

가장 안전한 방법은 라우터 전원을 Jetson과 같은 클린 도메인에서 따되 절연 또는 별도 레귤레이션을 적용하는 것인데, TDK 절연 컨버터의 18V 출력을 활용할 수도 있지만 Jetson과 라우터를 함께 공급하려면 TDK 컨버터의 용량을 확인해야 한다. IEA48004A180V 모델 사양을 정확히 파악해야 하는데, 입력 48V에 출력 18V이고 용량이 40W 정도인 것 같지만 정확한 전류 사양을 다시 확인해야 한다.

Jetson Orin Nano가 7~25W를 소비하고 라우터가 15W를 추가하면 총 40W 근처가 되어 TDK 컨버터의 여유가 부족할 수 있으므로, 더 깔끔한 방법은 라우터 전용으로 별도의 작은 48V→5V USB 절연 또는 비절연 벅 컨버터를 사용하는 것이다. 12V 레일을 쓰면 Dynamixel과 노이즈를 공유하게 되니 피해야 한다. 결국 라우터는 모터 레일에서 직접 전원을 따면 안 되고, Jetson 같은 클린 도메인에서 전용 레귤레이터로 따야 한다는 게 핵심인데, 사용자가 EMI를 신경 쓰는 만큼 이 부분을 강조해야 한다.

가장 깔끔한 방법은 48V에서 절연형 DC-DC 모듈로 5V를 만들어 라우터 USB-C에 연결하는 것이고, 간단하게 가려면 TDK 18V 출력이 여유 있으면 거기서 작은 벅 컨버터로 5V를 만드는 방법도 있다. 어느 쪽이든 라우터 입력단에 캐패시터와 필터를 달아서 안정적인 5V/3A를 보장해야 한다. 라우터는 통신 장비라 전압 강하에 민감하니까.

라우터 선택은 USB-C 5V 입력에 전력 소비가 적은 모델로 가야 하는데, Beryl AX(GL-MT3000) 같은 게 5V/3A USB-C를 지원한다.

Beryl AX의 전력 소비는 최대 15W 정도지만 보통 5~8W 수준이고, Beryl 7도 USB-C를 지원한다. WiFi 6이면 충분해서 WiFi 7은 필요 없다.

구체적인 연결은 절연형 DC-DC 모듈(Mean Well이나 TDK 같은 산업용, 또는 자동차용 48V→USB-C 모듈)로 5V/3A를 만들어서 USB-C 커넥터로 라우터에 연결하고, DC-DC 입력은 PDB의 빈 탭이나 출력 C에서 분기하면 된다.

Jetson 절연 컨버터 앞단의 48V에서 따서 별도 절연을 거쳐 Jetson과 라우터가 각각 깨끗한 전원을 받도록 하는 게 좋다. 6개 모터의 동시 회생제동 시 48V 레일이 60V 이상으로 튈 수 있으니, 48V→5V 모듈은 입력 정격이 60V 이상 또는 75V 정도인 제품을 선택하고 TVS 보호도 추가해야 한다.

라우터도 Jetson과 같은 수준의 노이즈/스파이크 보호가 필요하다. 최종적으로 Beryl AX(GL-MT3000) 라우터를 USB-C 5V/3A로 구동하되, 모터 레일에 직결하지 말고 절연형 DC-DC 컨버터를 거쳐 전원을 공급하는 방식으로 진행한다.

사용자가 구체적인 토폴로지를 제시했으니 라우터를 출력 C에 추가하는 형태로 간단히 보여주고, 회생 스파이크 대비 입력 정격과 TVS 보호를 강조하는 기술적 답변을 제공하자. 사용자가 6모터 회생제동 위험을 언급했으니 이걸 라우터 전원 설계와 연결해야 한다. 핵심은 라우터를 Jetson처럼 취급해서 모터/ODrive 레일과 분리하고, 48V에서 5V/3A로 변환하는 절연 DC-DC 컨버터를 써야 한다는 것 — 이 컨버터의 입력 정격이 회생 스파이크를 견딜 수 있어야 한다.

절연형 48V→5V 모듈은 PoE나 텔레콤 분야에서 흔하고, 비절연 와이드 입력 벅(8-60V 범위)도 옵션인데, Jetson이 절연을 썼으니 라우터도 절연하면 그라운드 루프와 노이즈 격리 이점이 있다. 비용과 공간 트레이드오프를 고려하되, 최소한 와이드 입력 범위와 입력 필터로 회생 스파이크를 처리해야 한다.

절연형 48V→5V DC-DC(텔레콤급, 36-75V 입력, 5V/3A 이상)를 권장하거나, TDK iEA의 18V 출력에서 소형 18V→5V 벅을 분기하는 방법도 있다. 후자는 TDK 용량이 충분하면(Jetson 25W + 라우터 15W = 40W 합산) 더 간단하지만, 60W급 모듈이 필요할 수 있다. TDK 데이터시트를 확인해야 정확한 정격을 알 수 있겠지만, 모델명이 약식 표기라 확실하지 않다. iEA 시리즈가 보통 9-40W 범위의 절연 DC-DC 컨버터인데, "IEA48004A180V"에서 "04"가 4A를 의미한다면 18V×4A=72W 정도일 것 같다. 사용자가 구체적인 모델명을 제시했으니 정격을 찾아서 알려주는 게 더 도움이 될 것 같다. 검색해보고 안 나오면 일반적인 조언으로 돌아가자.
```
This block is not supported on your current device yet.
```

iEA 시리즈 사양을 정리하고 있는데, 48-78W 범위의 격리형 DC-DC 컨버터로 18V부터 60V 또는 36V부터 75V 입력을 지원한다. 출력은 5V 15A에서 28V 2.67A 범위이고, iEA48004A180V 모델은 48V 입력군에 속한다.

iEA48004 모델은 약 72-78W급으로 추정되며 18V/4A 정도의 출력을 제공한다. Jetson Orin Nano는 최대 25W, Beryl AX 라우터는 최대 15W이므로 총 40W 정도로 충분한 여유가 있다. 따라서 18V 레일에서 라우터용 소형 벅 컨버터를 분기해도 용량상 문제없다.

다만 입력 정격이 36-75V인데, 12S LiPo 완충 시 50.4V에서 회생제동 시 60V 이상으로 올라갈 수 있고, 6개 BLDC 동시 회생 시 75V에 근접할 가능성이 있다. 이는 Jetson 컨버터도 같은 리스크를 가지고 있으며, 사용자가 이미 회생제동 위험을 인지하고 있다. 라우터 전원도 동일한 보호를 받게 된다.

라우터는 Beryl AX(GL-MT3000)로 USB-C 5V/3A 입력이면 충분하고, WiFi6+DFS 기능으로 충분하다. 전원 연결 시 핵심은 ODrive/BLDC 출력 레일에서 직접 따지 않는 것이다. 회생 스파이크와 스위칭 노이즈가 있기 때문이고, 라우터 같은 통신장비는 brownout과 노이즈에 취약하다. 따라서 라우터를 Jetson과 같은 "클린" 도메인으로 취급해서 출력 C에 연결하고, TDK iEA의 18V 출력에서 18V→5V/3A 소형 벅 컨버터를 분기해 라우터에 연결하는 방식이 간단하다. 데이터시트로 정확한 정격을 확인해야 한다.

또는 48V에서 별도의 절연 DC-DC 모듈(36-75V 입력, 5V/3A 출력)을 사용해 라우터에 전원을 공급하면 Jetson과 완전히 독립시킬 수 있다. 어느 경로든 입력이 75V 스파이크를 견뎌야 하므로, 48V 직입력 모듈의 입력 정격과 TVS 보호를 확인해야 한다.

Adding topology diagram... 방법 2가 노이즈 격리 측면에서 더 우수하지만, 방법 1에 작은 LC 필터를 추가하면 충분할 수 있다. 라우터는 USB-C 전원이 필요한데, DC-DC 출력을 5V USB-C로 변환해야 한다.

라우터의 USB-C 포트에 5V를 공급하려면 CC 핀에 5.1kΩ 풀다운 저항을 연결해 5V 기본값으로 인식시키거나, USB-C 브레이크아웃 모듈을 사용하거나, 기존 USB-C 케이블을 개조해서 직접 연결할 수 있다.

실제 구성은 Beryl AX 라우터(USB-C 5V/3A, WiFi6, DFS 지원)를 기준으로, 48V 전원에서 별도의 절연 DC-DC 컨버터(36-75V → 5V/3A)로 라우터에 독립적으로 전력을 공급하는 것이 권장된다. 이렇게 하면 Jetson iEA와 병렬로 구성하면서도 전원 도메인을 완전히 분리할 수 있다.

더 간단한 방식으로는 iEA의 18V 출력을 벅 컨버터로 5V로 낮춰 라우터에 공급할 수도 있는데, iEA가 약 70W 정도라 여유가 있지만 18V 레일을 공유하면서 노이즈가 발생할 수 있다는 점을 고려해야 한다.

회생 스파이크 대응으로는 입력 모듈이 75V 이상을 견딜 수 있어야 하고, 그 이상의 스파이크는 TVS나 클램프로 보호해야 한다. USB-C 공급 시에는 CC 핀에 5.1k 풀다운이 필요한데, USB-C 출력 모듈을 사용하면 이미 처리되어 있다.

이제 다이어그램을 그려보자.라우터는 앞서 말한 **Beryl AX(GL-MT3000)**가 이 용도에 맞다. USB-C 5V/3A 입력이라 로봇 전원에서 따기 쉽고, WiFi 6 + DFS면 충분하다. WiFi 7(Beryl 7)은 영상 5Mbps엔 과하고 전력만 더 먹는다.

전원 연결이 핵심인데, 원칙부터. **출력 A·B(ODrive/BL70200 peak 레일)에서는 절대 따지 마라.** 6개 BLDC 동시 회생제동 때 레일이 펌핑되고(완충 50.4V에서 60V+로) FOC 스위칭 노이즈가 그대로 올라탄다. 라우터는 통신 장비라 brownout과 노이즈에 약해서, 그 레일에 물리면 결정적 순간에 링크가 끊긴다. 라우터는 Jetson과 같은 클린 도메인, 즉 **출력 C에 붙이는 게 맞다.**

두 가지 길이 있다.

권장 — 독립 절연:
```
└─ 출력 C
     ├─ ODrive #3 → BL70200 ×2
     ├─ TDK iEA(절연) → 18V → Jetson
     └─ 별도 절연 DC-DC (48V→5V/3A) → router   ← 추가
```
Jetson과 라우터가 각자 절연 변환을 거쳐 ground·노이즈가 분리된다. EMI 관점에서 가장 깨끗하다. iEA와 같은 계열의 5V 출력 모델이나 소형 절연 brick 하나면 된다.

간단 대안 — iEA 18V 레일 공유:
```
└─ TDK iEA → 18V ─┬─ Jetson
                   └─ 18V→5V/3A 소형 벅 → router
```
iEA 시리즈는 48~78W급에 36~75V 입력, 18V 단일 출력이라, Jetson(Orin Nano 최대 25W) + 라우터(최대 15W) 합 40W면 용량 여유가 있다. 정확한 정격은 데이터시트로 확인해라. 다만 18V 레일을 공유하므로 18V→5V 벅 입력에 작은 LC 필터를 넣어 벅 스위칭 노이즈가 Jetson 쪽으로 역류하지 않게 해라.

어느 쪽이든 두 가지 챙겨라:

- **회생 스파이크.** iEA 입력 상한이 75V인데 6모터 동시 회생이면 거기 근접할 수 있다. Jetson 컨버터가 이미 같은 리스크를 안고 있으니, 메인 레일의 TVS/브레이크 저항 클램프가 75V 아래로 잡아주는지 확인해라. 별도 48V→5V 모듈을 쓸 경우도 입력 정격이 이 스파이크를 견뎌야 한다.
- **USB-C 공급 디테일.** Beryl AX는 USB-C 입력만 있고, USB-C는 CC 핀에 5.1kΩ 풀다운 2개가 있어야 소스를 5V default로 인식한다. "USB-C 출력" 명시된 모듈을 쓰면 처리돼 있고, 직접 만들면 CC 풀다운을 빼먹지 마라.

정리하면 라우터는 Beryl AX, 전원은 출력 C에서 별도 절연 DC-DC로 USB-C 5V/3A 공급이 기본값이다.

### User · 2026-06-14T09:35:11.186911Z

걍 jetson usb-c출력 전원 꽂으면 안되냐

### Claude · 2026-06-14T09:35:47.336524Z

사용자가 Jetson의 USB-C 포트에서 라우터 전원을 공급할 수 있는지 묻고 있다. Jetson Orin Nano 개발 키트의 USB-C는 데이터 포트(USB 3.2)이지 전원 출력 포트가 아니므로 불가능하다.

USB 포트는 보통 5V만 공급하고 전류 제한이 있는데, Beryl AX 라우터는 5V/3A(15W)를 필요로 한다. Jetson의 USB 포트가 이 전류를 안정적으로 공급하기 어렵고, 라우터에 전력을 빼앗기면 Jetson 자체의 연산(YOLO+TensorRT 풀로드 25W)에서 전력 부족으로 성능 저하나 불안정이 생길 수 있다. USB 포트 전원은 핫플러그와 전류 제한으로 인해 라우터에 안정적인 전력을 보장할 수 없어서, 라우터 재부팅 시 통신 끊김으로 미션 실패 위험이 있다.

라우터 부팅 시 순간 전류 튐이 Jetson USB 포트의 과전류 보호를 트립시켜 포트가 차단되거나 Jetson이 리셋될 수 있다. 결론적으로 Jetson USB-C 포트로 라우터에 전력을 공급하는 것은 권장하지 않는다. 포트가 데이터용이라 전원 출력 능력이 약하고, 전력 부족과 불안정성으로 인해 통신 생명줄을 위험에 빠뜨리기 때문이다.

대회에서 영상이 끊기면 원격주행 구간이 실패하므로 라우터 전원은 깨끗하고 독립적이어야 한다. 책상에서 잠깐 테스트할 때는 작동할 수도 있지만, 실전에서 YOLO 풀로드 환경에서는 절대 안 된다. 별도 DC-DC 컨버터 하나가 보험이다.

Orin Nano의 USB-C 포트가 전원 공급 역할을 할 수 있는지는 캐리어보드마다 다를 수 있지만, 일반적으로 USB 포트 전원은 전류 부족, Jetson 연산 전력 예산 잠식, inrush나 트립으로 인한 리셋 문제가 생길 수 있다.

그래서 통신 라인으로는 위험하고, 별도의 DC-DC 컨버터를 사용하는 게 맞다.비추다. 책상에서 잠깐 테스트면 몰라도, 대회 실전에선 세 가지가 문제다.

첫째, Jetson USB 포트는 전원 출력용이 아니라 데이터 포트라 source 전류가 약하다. Beryl AX는 peak 5V/3A(15W)를 요구하는데 포트가 그걸 못 줘서 brownout 나면 라우터가 리셋된다.

둘째, USB 버스에서 빼가는 전력은 Jetson 자체 전력 예산에서 나온다. YOLO를 TensorRT로 풀로드 돌리는 25W 모드에선 연산이랑 경합해서 throttle 걸리거나 불안정해진다. 모터가 안 도는 구간인데도 라우터 때문에 추론이 느려지는 식이다.

셋째, 라우터 부팅 inrush로 포트 과전류 보호가 트립되면 라우터랑 Jetson이 같이 리셋된다. 영상 송신원(Jetson)과 링크(라우터)를 한 전원 트립에 묶는 셈이다.

핵심은 라우터가 원격주행의 생명줄이라는 거다. 끊기면 그 구간 실패니까, 전원만큼은 Jetson 전류 예산에 의존시키지 말고 별도 DC-DC로 독립시키는 게 맞다. 부품 하나 값이 보험이다.

### User · 2026-06-14T09:37:07.223975Z

└─ TDK iEA → 18V ─┬─ Jetson
                   └─ 18V→5V/3A 소형 벅 → router

이렇게 간다 치고 여기 달만한 벅컨버터랑 필터 모듈 추천좀. usb-c 아웃풋 되는걸로

### Claude · 2026-06-14T09:38:55.617777Z

사용자가 18V 전원에서 5V/3A 출력을 제공하는 벅 컨버터를 찾고 있고, USB-C 포트와 필터 모듈이 있는 제품을 원하고 있다. 이건 구체적인 부품 추천이 필요한데, 제품 시장이 자주 변하고 SEO 영향이 크기 때문에 신중하게 접근해야 한다.

요구사항을 정리하면 입력은 18V 공칭이지만 여유를 위해 8-22V 또는 6-32V 범위를 지원하는 게 좋고, 출력은 5V 고정 3A USB-C면 충분하다. 입력과 출력 사이에 필터를 두어 벅 컨버터의 스위칭 노이즈가 Jetson으로 전파되지 않고 모터 노이즈가 라우터로 들어오지 않도록 격리해야 한다. 차량용 12-24V USB-C 충전 모듈이 이 요구사항을 만족할 가능성이 있는데, 18V 입력을 지원하고 방진 설계가 되어 있다. 사용자가 EMI 필터 설계를 이미 알고 있으니 구체적인 제품 추천과 사양만 제시하면 될 것 같다. 차량용/산업용 벅 모듈 중 USB-C 출력이 있는 제품들을 찾아봐야겠는데, Pololu 같은 곳의 스텝다운 레귤레이터들을 확인해보자. 두 가지 접근 방식을 제시하는 게 낫겠다. 첫 번째는 USB-C PD 출력이 내장된 산업용/차량용 벅 모듈로 가장 간단하고, 두 번째는 핀 출력 벅 컨버터에 USB-C 브레이크아웃을 조합하는 방식으로 더 견고하고 커스터마이징 가능하다. 입력측 필터링(LC π 필터와 CMC)도 함께 고려해야 한다. 실제로는 추천 기준이 더 중요한데, 입력 범위가 9-36V 정도로 넓어야 18V 공급 변동과 스파이크에 대응할 수 있고, 출력은 5V/3A 연속이지만 여유 있게 5A급이 안정적이다. USB-C는 기본 5V 협상이면 충분하지만, 일부 PD 충전기가 부하 없을 때 5V에서 QC로 9V로 올릴 수 있다는 점을 고려해야 한다.

필터링 설계로는 벅 입력단에서 모터 노이즈와 iEA 출력 리플을 차단하기 위해 페라이트 비드나 CMC와 전해/세라믹 벌크 캡을 사용하고, 출력단에서는 벅 스위칭 리플을 제거하는 LC 또는 페라이트 필터를 적용하되, 라우터 내부 레귤레이터도 고려해야 한다.

18V 공유 레일이 양방향 노이즈(모터 쪽에서 들어오는 노이즈와 벅에서 나가는 스위칭 노이즈)에 노출되므로 입력 필터링이 더 중요하다. 사용자가 이미 π필터와 CMC 설계 경험이 있으니 직접 LC를 구성하는 게 맞겠지만, 상용 EMI 필터 모듈(자동차용 DC 라인 필터, Würth나 Murata의 DC-DC 입력 필터 등)도 옵션으로 고려할 수 있다.

이제 USB-C 출력 벅 컨버터 모듈을 찾아보자 — 5V 3A 정도 출력에 광범위 입력 전압을 지원하는 제품을 검색해야 한다.
```
This block is not supported on your current device yet.
```

제품들을 살펴보니 대부분 9-36V 입력에 5V/3A USB-C 출력을 제공하는 일반적인 모듈들이다. 핵심은 사양 기준을 제시하고 주의점을 정리하는 것인데, iEA 18V 출력이 9-36V 범위의 중간값이므로 충분한 마진이 있다.

가장 중요한 주의점은 QC3.0/PD 지원 여부다. 많은 제품이 QC나 PD를 지원하면서 출력 전압을 협상으로 올리는데, Beryl AX는 5V만 받아야 한다. 라우터가 QC 핸드셰이크를 하지 않으면 5V 기본값이 유지되지만, QC 트리거 오작동으로 9V가 나가면 라우터가 손상될 수 있다. SSLHONG 같은 제품은 QC3.0 표기가 있어서 위험하고, UCTRONICS U6223처럼 5-5.3V 고정 출력(QC 미지원) 모듈이 훨씬 안전하다.

품질과 신뢰성 측면에서 $1-8 범위의 저가 중국산 모듈은 대회용으로 불안정하다. Mean Well이나 Pololu 같은 산업용 브랜드 모듈에 USB-C 브레이크아웃을 추가하는 방식이 더 신뢰할 수 있지만, 사용자가 USB-C 커넥터 작업을 원하지 않을 수 있다.

현실적으로는 보호회로와 에폭시 포팅이 있는 괜찮은 제네릭 모듈을 선택하거나, 산업용 벅 컨버터에 USB-C 커넥터를 직접 납땜하는 절충안을 고려할 수 있다. iEA 18V 레일을 공유하면 벅 입력의 노이즈를 양방향으로 격리해야 한다.

사용자가 전문가이니 솔직하게 말하면, 제네릭 USB-C 5V/3A 벅은 많고 9-36V 입력 사양만 맞으면 대부분 작동한다. 다만 (a) PD/QC 없이 고정 5V, (b) 안정성을 위해 3A보다는 5A급, (c) 보호회로가 있는 제품을 선택하는 게 중요하다.

대회 신뢰성이 중요하면 Pololu나 Recom 같은 산업용 벅에 USB-C 브레이크아웃을 직접 만드는 게 더 견고하다. 18V 레일 공유 시 모터 노이즈가 라우터로, 벅 스위칭이 Jetson 18V로 역류하는 걸 막으려면 벅 입력단에 CM 초크와 π 필터를 달아야 한다. 사용자가 이미 π 필터나 CMC 설계에 익숙하니 자동차 DC 라인 EMI 필터 같은 상용 모듈을 쓰거나 페라이트 비드와 캡을 직접 구성하면 된다.

입력단에 페라이트 클램프, 100µF 전해 캡, 0.1µF 세라믹 캡을 달고 출력단에도 페라이트와 캡을 붙인다. 라우터는 자체 레귤레이션이 있어서 출력측은 덜 중요하고 공유 레일인 입력측이 핵심이다. Beryl AX의 USB-C 5V는 CC 핀에 5.1k 풀다운이 필요한데, USB-C 출력 모듈은 이미 CC 처리가 되어 있다.

답변은 간결하게: 일반적인 9-36V→5V/3A 벅 모듈(UCTRONICS U6223 같은 fixed 5V 타입)로 충분하지만, 대회 생명줄이면 산업용 벅 + USB-C 브레이크아웃으로 5A급 여유를 두는 게 더 안전하다. QC나 PD 모듈은 피해야 한다—전압 협상 중 9V로 튈 수 있어서 라우터가 죽을 수 있다. 필터는 18V 공유 레일이라 양방향이므로 벅 입력에 CMC와 LC 파이 필터를 달고 페라이트와 캡을 붙인다. 5A 정도면 충분한 여유가 있겠네. 신뢰성이 중요하다면 generic 모듈보다는 산업용 벅 컨버터에 USB-C 브레이크아웃을 조합하는 게 더 안전할 것 같다. 양방향 격리를 위해 공유 18V 레일에 CMC와 π 필터를 적용해야 한다.벅 모듈부터. 9–36V 입력에 5V/3A USB-C 출력 모듈은 차량/산업용으로 흔하다. 18V는 그 범위 한가운데라 마진 충분하다. 다만 고를 때 두 가지 주의.

첫째, **QC/PD 협상형을 피하고 5V fixed 출력을 써라.** 검색에 흔히 나오는 "QC3.0 Type-C" 모듈들은 입력 8–35V에 출력이 5V/3A, 9V/2A, 12V/1.5A로 협상되는 식이다. 라우터가 PD 핸드셰이크를 안 하면 보통 5V default로 떨어지지만, 트리거 오작동으로 9V가 나가면 라우터가 죽는다. UCTRONICS U6223처럼 9–36V 입력에 5–5.3V fixed로 떨어뜨리는 dumb 출력 모듈이 안전하다. 그리고 라우터 peak가 3A니 마진 위해 5A급을 골라라.

둘째, 이게 원격주행 생명줄이라는 점. $1~8짜리 generic 차이나 모듈은 inrush·발열·보호회로가 들쭉날쭉이다. 신뢰성을 원하면 **산업용 벅(Pololu D24V 계열, Recom R-78 류 등 핀 출력) + USB-C 브레이크아웃 보드**를 조합하는 게 generic USB-C 일체형보다 견고하다. 너가 직접 짜는 게 익숙하면 이쪽이 낫다. 어느 쪽이든 epoxy potting + 과전류/과열 보호 있는 걸로.

필터는, 18V 레일을 Jetson과 공유하니 **양방향 격리**가 핵심이다 — 모터 노이즈가 라우터로 들어가는 것과, 벅 스위칭 리플이 Jetson 18V 입력으로 역류하는 것 둘 다 막아야 한다. 너가 이미 쓰는 CMC + π 구성 그대로 가면 된다. 벅 입력단에 common-mode choke + LC π필터, 대략:

- CMC: 공통모드, 수 mH급
- 직렬 페라이트 비드 + 벌크 100µF 전해 + 0.1µF 세라믹 (입력)
- 출력단은 라우터 자체 레귤레이션이 있어 덜 critical하지만, 페라이트 + 22~47µF 하나 더 넣으면 깔끔

상용 모듈로 갈 거면 자동차용 DC 라인 EMI 필터 보드가 있긴 한데, 너 수준이면 직접 LC 짜는 게 값·성능 다 낫다. 별도 "필터 모듈" 살 필요는 없어 보인다.

정리: 9–36V→5V fixed USB-C 벅(5A급, QC/PD 아닌 것), 입력단에 CMC+π, 공유 레일 양방향 격리.

### User · 2026-06-14T09:40:50.469287Z

필터는 직접 짠다 치고, 다른 부품들 링크좀 줘봐

### Claude · 2026-06-14T09:41:49.693910Z

사용자가 구체적인 부품 구매 링크를 요청하고 있는데, 나는 신뢰할 수 있는 링크를 제공하기 어렵다. Amazon이나 eBay 링크는 재고 변동이 있고 한국 사용자의 지역 문제도 있으며, 제품의 진위를 보장할 수 없기 때문이다.

대신 정직하게 접근해야 한다. 제조사 공식 페이지(Pololu, Recom, GL.iNet, TDK 등)는 안정적이므로 제공할 수 있고, 검색에서 나온 정확한 제품명과 브랜드를 알려주면 사용자가 직접 소싱하는 게 낫다. 라우터 같은 경우 GL.iNet Beryl AX는 공식 스토어 링크가 있고, 벅 컨버터는 Pololu 같은 신뢰할 만한 산업용 브랜드의 공식 사이트를 제시할 수 있다. 한국 구매처들을 고려하면서, 정직한 접근이 필요하다는 걸 깨닫고 있다. 실시간 링크를 보장할 수 없으니 대신 공식 소스와 정확한 모델명, 검색어를 제공하고 한국 소싱 팁을 함께 주는 게 낫겠다. 직접 확인하지 않은 링크를 주는 건 위험하니까.

GL.iNet 라우터는 공식 스토어 링크를 중심으로, Pololu 모터는 공식 사이트나 Mouser/Digikey 같은 신뢰할 수 있는 유통처를 추천하고, 한국에서는 마우저/디지키 코리아나 디바이스마트, 알리익스프레스 같은 곳들을 활용하도록 안내하는 게 좋겠다. 몇 가지 안정적인 링크는 제시하되, 재고와 지역별 가용성은 사용자가 직접 확인해야 한다는 점을 명확히 해야 한다. 완성된 부품 리스트를 정리하면 라우터, 벅 컨버터(신뢰성 우선 또는 간편한 옵션), 필터 부품들인데, 링크는 공식 사이트와 한국 유통처 중심으로 제시하되 정확성 한계를 명시하겠다. 결국 사용자에게 가장 실질적인 도움은 정확한 모델명과 검색 키워드, 그리고 한국에서 구매 가능한 곳을 알려주는 것이다. 구체적인 모델명과 신뢰할 수 있는 소싱처를 제시하되, 주요 공식 링크만 조심스럽게 포함하는 방식으로 가자. 검색 결과에서 찾은 GL.iNet 공식 스토어, Pololu, 그리고 일반적인 전자부품 판매처들의 URL을 참고해서 사용자가 직접 확인할 수 있도록 안내하는 게 낫겠다.솔직히 라이브 구매 링크는 재고·지역(한국 배송)·진위가 계속 바뀌어서 "이거 클릭해서 사라"로 보장하긴 어렵다. 그래서 검색되는 정확한 모델명/키워드 + 신뢰할 소싱처로 준다. 메이저 브랜드 공식만 링크 걸고, generic은 모델명으로.

**라우터 — GL.iNet Beryl AX (GL-MT3000)**
공식: store.gl-inet.com 에서 "Beryl AX" ([제품 페이지](https://store-us.gl-inet.com/products/beryl-ax-gl-mt3000-pocket-sized-wi-fi-6-wireless-travel-gigabit-router))
한국이면 알리익스프레스 GL.iNet 공식스토어나 네이버/쿠팡에 "GL-MT3000" 검색이 배송 빠르다. 모델 코드 GL-MT3000으로 검색해야 정확하다.

**벅 — 두 갈래**

신뢰성 우선(권장): 산업용 핀출력 벅 + USB-C 브레이크아웃 조합.
- Pololu 5V 스텝다운 레귤레이터 (pololu.com, "5V step-down" 카테고리). 18V 입력 받는 D24V 계열 중 3A 이상.
- 또는 Recom R-78 시리즈 / Mean Well — 마우저·디지키 코리아에서 정품.
- USB-C 출력은 Adafruit/SparkFun USB-C 브레이크아웃에 5V 직결(CC 5.1kΩ 풀다운 내장된 것).

간편: generic 일체형. 검색어 **"9-36V to 5V 3A USB-C buck fixed"** (QC/PD 말고 fixed). 디바이스마트·엘레파츠·알리에서 나온다. 앞서 본 UCTRONICS 류처럼 5–5.3V fixed 출력인 걸 골라라. 다만 대회 생명줄이면 이쪽은 예비로만.

**필터 부품(직접 짜니까 — 정품 부품상에서)**
마우저·디지키 코리아 또는 디바이스마트:
- Common-mode choke: Würth WE-CMB 시리즈, TDK ACM 시리즈 (수 mH, 정격전류 3A↑)
- 페라이트 비드: Würth/Murata, 18V 라인 정격
- 캡: 입력 100µF 전해(정격 35V↑, 회생 스파이크 고려하면 50V) + 0.1µF MLCC, 출력 22~47µF

소싱 우선순위는, 라우터는 GL.iNet 공식, 벅·필터 부품은 마우저/디지키 코리아(정품·납기 확실)가 깔끔하다. generic 모듈만 알리/디바이스마트.

벅 모델 후보 좁혀지면 입력 정격이 회생 스파이크(레일 펌핑) 견디는지 같이 봐줄게.

### User · 2026-06-14T09:57:06.885877Z

필터는 어떻게 뽑지? pcb로?

### Claude · 2026-06-14T09:57:58.276016Z

사용자가 라우터 전원 라인에 달 CMC와 π 필터를 어떻게 물리적으로 구현할지 묻고 있다. 별도 PCB로 만들지, 아니면 만능기판이나 인라인 같은 다른 방식이 나을지 고민 중이다.

필터 자체는 부품이 적고 전류도 3A 정도로 크지 않아서 여러 방법이 가능하지만, EMI 필터는 레이아웃이 중요해서 공중배선이나 만능기판보다는 PCB가 성능상 유리하다. 사용자가 이미 메인 전원 시스템 PCB를 설계 중인 것 같으니, 이 라우터 필터도 별도 작은 PCB로 뽑거나 기존 설계에 통합하는 게 가장 깔끔할 것 같다. JLCPCB가 맞다. 핵심은 별도 PCB로 분리할 가치가 있는지 판단하는 것인데, 이미 메인 전원 PCB를 작업 중이면 거기 통합하는 게 BOM과 신뢰성 면에서 낫다. 별도 보드는 커넥터 접점이 추가되어 고장점만 늘어난다. 벅 모듈을 구매할 거면 그 입력 앞에 필터를 두고, 벅까지 자체 설계하면 둘을 한 보드에 담으면 된다.

라우터 전원(필터+벅)을 메인 전원 분배 보드의 한 브랜치로 통합하는 게 가장 합리적이다. 별도 PCB는 불필요하고, 이미 로직 브랜치에서 하던 CMC+π 패턴을 라우터 브랜치에도 그대로 적용하면 된다. 굳이 분리한다면 작은 2층 PCB를 KiCad로 그려서 JLCPCB에 보내면 되는데, 부품이 적어서 빠르게 완성된다.

EMI 필터는 레이아웃이 성능을 좌우하므로, 입력과 출력을 물리적으로 분리해서 필터를 우회하는 결합을 방지하고, CMC 입출력 트레이스가 교차하거나 근접하지 않도록 주의해야 한다. 캡 리턴은 짧게, 그라운드 포어는 넓게 하고, 3A 전류라 2온스 동박이면 트레이스 폭이 충분하다. Perfboard나 공중배선도 동작하지만 고주파 감쇠가 레이아웃에 따라 들쭉날쭉해서 재현성을 위해선 PCB가 낫고, 1회성 테스트면 perfboard도 괜찮다. 결국 이 필터는 단순 회로지만 "필터"라는 특성상 PCB 레이아웃이 곧 성능이다. 작은 2층 PCB를 JLCPCB에서 발주하는 게 낫겠다. perfboard는 프로토타입 테스트 정도는 되지만 EMI 필터는 레이아웃이 성능을 좌우하니까 본제품은 제대로 된 PCB가 필요하다. 핵심 레이아웃 포인트 4가지를 텍스트로 설명하면 충분할 것 같다.

사용자가 묻는 건 제조 방법이지 회로 다이어그램을 원하는 게 아니네.따로 PCB 발주하는 것보다, 너 이미 작업 중인 전원 분배/로직 브랜치 보드의 한 브랜치로 통합하는 게 낫다. 별도 보드로 빼면 커넥터 접점이 하나 더 생기는데 그게 곧 고장점이고, 진동 많은 로봇에선 더 그렇다. 메모리상 로직 브랜치에 이미 CMC+π를 깔고 있으니, 라우터 전원도 같은 패턴으로 그 보드에 브랜치 하나 더 따면 된다. 벅까지 직접 설계하면 필터+벅이 한 보드에 들어가서 제일 깔끔하다.

굳이 분리해야 하는 상황이면(generic 벅 모듈을 살 거라 그 앞단만 필요하다든가) 2층 소형 PCB로 KiCad 그려서 JLCPCB 발주가 무난하다. 부품 몇 개라 금방 뽑힌다.

perfboard나 공중배선도 3A·저주파면 동작은 하는데, **EMI 필터는 레이아웃이 곧 성능**이라 본번엔 권하지 않는다. 기생 인덕턴스랑 입출력 결합 때문에 고주파 감쇠가 들쭉날쭉해지고, 정작 막으려던 모터 스위칭 노이즈가 필터를 건너뛰어 버린다. 책상 테스트용 1회성이면 perfboard로 빠르게 확인하는 정도는 괜찮다.

PCB로 갈 때 성능 좌우하는 레이아웃 포인트:

- 입력단과 출력단을 물리적으로 떨어뜨려라. 가까우면 노이즈가 필터를 우회해 결합한다.
- CMC 입출력 트레이스를 나란히/교차로 두지 마라. 같은 이유로 결합 생긴다.
- 캡의 ground return을 최대한 짧게, 넓은 ground pour 위에. 캡 성능은 return path가 결정한다.
- 3A니까 전류 경로 트레이스 폭 충분히. 2oz 동박이면 여유롭다.

정리하면 별도 발주보다 메인 전원 보드 통합이 1순위, 분리하면 2층 PCB, perfboard는 테스트 한정이다.

### User · 2026-06-14T10:06:23.959997Z

필터 없으면 어떻게되지

### Claude · 2026-06-14T10:07:15.946131Z

사용자가 EMI 필터 없이 라우터 전원 라인에서 어떤 문제가 발생하는지 묻고 있다. 18V 레일에서 벅 컨버터를 거쳐 5V USB-C로 라우터에 공급되는데, 이 경로에 모터 노이즈와 벅 스위칭 노이즈가 역류하면서 필터의 필요성을 언급하고 있다.

라우터 자체는 통신 장비라 전원 노이즈가 크면 WiFi 무선 성능이 저하되고 — 노이즈가 RF 프론트엔드와 PLL에 결합되면서 phase noise가 증가해 처리량이 떨어지고 패킷 손실이 생긴다. 심한 경우 벅 입력이 흔들려 라우터가 리셋될 수도 있지만, 라우터 자체 입력 레귤레이션이 있어서 RF 성능 저하가 더 흔한 증상이다. 벅 스위칭 노이즈가 18V 레일로 역류하면 Jetson도 영향을 받는데, iEA 뒤라 어느 정도 보호되지만 노이즈가 심하면 카메라 USB나 아날로그 센서 같은 주변 센서에 문제가 생길 수 있다. 모터의 BLDC FOC 스위칭과 회생제동 노이즈도 라우터로 직접 타고 들어간다.

회생제동 중에 48V 레일이 펌핑되면서 iEA 입력이 흔들려 18V 출력이 변동하고, 이게 벅을 거쳐 라우터 전원에 dip이나 spike를 만든다. 모터 부하가 급격히 변할 때 라우터 RF 성능이 순간 저하되거나 리셋될 수 있는데, 특히 험지나 장애물 구간에서 모터가 풀로드로 가속하거나 회생제동할 때 영상이 끊기는 게 가장 위험하다. 결국 필터가 없을 때의 진짜 문제는 라우터가 켜지지 않는 게 아니라 조종이 가장 필요한 순간에 통신이 끊기는 것이다.

책상에서 모터를 안 돌릴 때는 멀쩡하다가 대회장 험지에서만 영상이 깨지는 패턴이 생기는데, 이건 재현과 디버깅이 정말 어렵다. 라우터와 Jetson이 각각 레귤레이션을 가지고 있어서 필터 없이도 모터 약부하 환경에선 동작하기 때문에 사람들이 "필터 없어도 괜찮네?"라고 착각하기 쉽지만, 회생제동과 풀로드가 동시에 일어나는 최악의 상황에서 문제가 터진다.

필터가 절대 필수는 아닐 수 있지만, 이 시스템의 특성상(6개 모터 회생, 공유 레일) 생략하면 간헐 장애 위험이 크고 그게 대회 결과를 좌우할 수 있다. 비용 대비 보험으로 다는 게 맞는데, 정확히 얼마나 심할지는 모터 노이즈를 실제로 측정하지 않으면 알 수 없다. 부하, 레이아웃, 접지 상태에 따라 달라지기 때문에 확실한 판단이 어렵다.

사용자가 원하는 건 확신 없으면 확신 없다고, 추측이면 추측이라고 말하는 것. 그래서 정리하면: 필터 없어도 라우터와 Jetson 자체 레귤레이션이 어느 정도 막아줘서 조용한 상황에선 멀쩡할 수 있지만, 모터 풀로드나 회생 순간에만 RF 성능 저하, 패킷 손실, 심하면 리셋이 발생할 수 있다. 특히 험지나 장애물 구간처럼 조종이 가장 중요한 순간에 영상이 깨지는 패턴이 나타나는데, 책상에선 재현이 안 되고 대회장에서만 터진다.

모터 노이즈가 최악인 이유는 실제 부하, 접지, 레이아웃에 따라 얼마나 심할지 미리 알 수 없기 때문에 보험 차원에서 필터를 다는 게 합리적이다. 증상 우선순위로 보면 WiFi 처리량 저하와 패킷 손실로 인한 영상 끊김이 가장 흔하고 직접적이고, 그 다음이 회생 스파이크로 인한 라우터 brownout이나 리셋, 마지막으로 Jetson 쪽 노이즈 역류가 카메라나 센서에 영향을 미치는 경우가 있다.

핵심은 이거다: 필터 없어도 평소엔 멀쩡할 수 있지만, 모터가 세게 도는 순간에만 영상이 깨진다 — 하필 조종이 제일 필요한 험지 구간에서. 그리고 책상 테스트론 절대 안 잡힌다.바로 안 터질 수도 있다. 라우터랑 Jetson 둘 다 입력에 자체 레귤레이션이 있어서, 모터가 약하게 돌거나 정지한 잔잔한 상태에선 필터 없이도 멀쩡히 돌아간다. 그래서 "어, 필터 없어도 되네" 하고 착각하기 쉽다.

문제는 **모터가 세게 도는 순간에만 간헐적으로 터진다**는 거다. 6개 BLDC가 풀로드로 돌거나 회생제동으로 48V 레일이 펌핑되면, 그 스파이크·노이즈가 공유 18V 레일 타고 벅을 거쳐 라우터에 들어간다. 그때 일어나는 일, 흔한 순서대로:

- WiFi RF 프론트엔드/PLL에 전원 노이즈가 결합해서 throughput이 떨어지고 패킷 손실이 늘어난다 → 영상이 깨지거나 멈춘다. 가장 흔하고 직접적인 증상.
- 회생 스파이크가 크면 벅 입력이 흔들려 라우터가 brownout·리셋. 그러면 그 구간 영상은 완전히 날아간다.
- 벅 스위칭 노이즈가 18V로 역류해 Jetson 쪽 카메라 USB나 센서에 영향. 덜 흔하지만 가능.

진짜 골치는 타이밍이다. 모터 풀로드는 험지·장애물 구간에서 걸리는데, 그게 바로 조종자가 영상 보고 정밀하게 몰아야 하는 순간이다. 정작 제일 필요할 때 끊긴다. 게다가 책상에서 모터 안 돌리고 테스트하면 100% 정상으로 나와서, 대회장 가서야 터지고 재현·디버깅이 지옥이다.

솔직히 얼마나 나쁠지는 너 시스템의 모터 노이즈 스펙트럼·접지·레이아웃에 달려서 실측 전엔 확언 못 한다 — 거의 문제없을 수도, 구간마다 끊길 수도 있다. 다만 변수가 회생 6모터라 다운사이드가 크고, 필터는 부품 몇 개 값이라 보험으로 까는 게 합리적이다. 안 깔 거면 최소한 리허설 때 모터 풀로드 + 회생 거는 상태에서 영상 안정성을 꼭 확인해라.

### User · 2026-06-14T10:10:27.218061Z

pcb없이 그냥 선납땜으로 대충 만드는건? 절연처리는 확실히 한다 치고

### Claude · 2026-06-14T10:11:14.205507Z

사용자가 PCB 없이 공중배선으로 필터를 만드는 것에 대해 다시 묻고 있는데, 절연처리는 확실히 하겠다고 한다. 내가 이전에 공중배선은 레이아웃에 따라 EMI 필터 성능이 불안정하지만 테스트용으로는 괜찮고 본격적으로는 PCB를 권장한다고 했으니, 이제 그 조건 하에서 사용자의 제안을 다시 평가해야 한다.

절연처리와 필터 성능은 별개 문제다. 절연을 잘하면 단락이나 감전 같은 안전 문제는 해결되지만, 그것이 필터의 고주파 성능을 보장하지는 않는다. 핵심은 레이아웃인데, 선납땜 자체가 불가능한 건 아니고 리드를 짧게 유지하고 입출력 선을 분리하며 그라운드 리턴을 굵게 하고 부품을 단단히 고정하면 충분히 동작할 수 있다. 이 필터는 모터 스위칭이나 벅 컨버터 주파수 대역이라 그렇게 극도로 높은 주파수가 아니기 때문에 선납땜으로도 실제로 작동 가능하다.

다만 로봇 환경의 진동과 충격을 고려하면, EMI 성능보다는 선납땜의 기계적 신뢰성이 더 큰 문제가 될 수 있다.

공중배선 부품이 흔들려서 리드가 피로 파괴되거나 납땜이 크랙되면 단선되고, 험지에서 끊기면 영상도 끊긴다. 그래서 PCB를 권하는 더 근본적인 이유일 수 있다. 다만 부품을 에폭시나 핫멜트로 단단히 고정하거나 작은 perfboard에 올려서 보드째 마운트하면 기계적 안정성을 확보할 수 있다.

EMI 성능은 선납땜도 리드를 짧게 유지하고 입출력선을 분리하면 이 주파수 대역에선 충분하다. 진짜 리스크는 기계적 단선인데, 절연만으로는 못 막으니 부품 고정이 핵심이다. Perfboard 조각을 사용하면 PCB 발주 없이도 ground 면을 어느 정도 확보하면서 기계적으로 더 안정적인 구성을 만들 수 있다.

사용자가 비용과 수고를 줄이려는 방향으로 가고 있는데, 선납땜까지는 괜찮지만 그 이상 대충하면 기계적 리스크가 커진다는 점을 명확히 해야겠다.

절연과 필터 성능은 별개고, 이 주파수 대역에선 선납땜도 충분한 EMI 감쇠를 낼 수 있다 — 다만 리드를 짧게, 입출력선을 분리하고, 그라운드를 굵고 짧게 해야 한다. 진짜 문제는 EMI가 아니라 로봇 진동으로 인한 공중배선 단선이고, 이건 절연으로 못 막는다. 그래서 공중배선이라도 핫멜트나 에폭시로 고정해야 한다.절연은 안전(단락·합선) 문제고, 필터가 잘 듣느냐랑은 별개다. 그건 레이아웃이 좌우한다.

EMI 성능만 보면 선납땜으로도 충분하다. 너가 막으려는 게 모터 스위칭이랑 벅 리플인데 대역이 수십 kHz~수 MHz라, 수십 MHz 이상 RF 필터처럼 기생 성분에 예민하지 않다. CMC+페라이트+캡 선납땜이면 의미 있는 감쇠가 나온다. "대충"의 함정은 두 가지뿐이다 — 리드를 길게 빼는 것(긴 리드 = 기생 인덕턴스라 고주파에서 캡이 무력화됨)과 입력선·출력선을 한 다발로 묶는 것(노이즈가 필터를 건너뛰어 커플링됨). 이 둘만 피하고 ground return을 짧고 굵게 가면 된다.

진짜 약점은 EMI가 아니라 기계적 신뢰성이다. 로봇은 진동·충격 환경이라 공중배선 부품이 흔들리면서 리드 피로 파괴, 납 크랙, 단선이 난다. 그게 그대로 영상 끊김이고, 절연처리로는 단선을 못 막는다. 험지 구간에서 한 번 끊기면 끝이다.

그래서 선납땜으로 갈 거면 부품을 공중에 매달지 말고 핫멜트나 에폭시로 단단히 굳혀라. 더 나은 절충은 만능기판 조각에 올려 납땜하고 그 보드를 케이스에 고정하는 거다. PCB 발주는 안 해도 되면서 기계적으로 훨씬 안정적이고, ground 면도 어느 정도 확보된다. 순수 공중배선보단 이게 낫다.

정확한 감쇠량은 너 노이즈 스펙트럼 실측 전엔 단언 못 하지만, 이 대역에선 만능기판 선납땜이 성능·신뢰성 균형이 제일 좋다.

### User · 2026-06-17T18:30:43.828116Z

라우터 좀더 싼걸로 없냐

### Claude · 2026-06-17T18:32:02.264668Z

사용자가 Beryl AX보다 더 저렴한 라우터를 찾고 있네. 이 용도에 필수적인 요구사항들을 다시 정리해보면 5GHz 대역폭, 채널 수동 설정 기능, 그리고 USB-C나 저전압 DC 전원 지원이 필요한데 로봇 전원 시스템과 호환되어야 한다.

GL.iNet의 저가 라인업을 살펴보니 Mango나 Shadow 같은 초저가 모델들은 2.4GHz만 지원해서 대회장 혼잡을 피할 수 없어 제외된다. Opal은 5GHz를 지원하고 가격도 괜찮지만 커스텀 펌웨어라 채널 제어가 제한적일 수 있다. 결국 5GHz는 포기할 수 없는 필수 요소고, DFS나 WiFi6 같은 기능들은 비용 절감을 위해 양보할 수 있을 것 같다. GL.iNet이 5GHz + 채널 고정 + 저전압 DC 전원을 동시에 만족하는 거의 유일한 선택지라는 걸 깨달았고, 일반 공유기는 AC 어댑터와 크기 때문에 로봇 탑재에 부적합하다. 중고 GL.iNet도 고려할 수 있지만, WiFi 동글을 노트북에 연결해서 Jetson과 연동하는 방식도 생각해볼 만하다.

5GHz를 포기할 수 없는 이유는 대회장의 혼잡한 무선 환경 때문인데, 2.4GHz만 지원하는 초저가 모델들은 쓸 수 없다. GL.iNet 라인업 중에서는 Opal이나 구형 Beryl 같은 WiFi5(ac) 모델로 내려가면 가격을 낮출 수 있고, 영상 전송에는 충분하다. 다만 채널을 수동으로 고정할 수 있는 자유도가 중요한데, OpenWrt 기반인 Beryl이나 Slate가 Opal보다 제어가 더 자유로워서 혼잡한 환경에서 채널 회피가 가능하다는 게 핵심이다.

DFS는 포기해도 되지만, 일반 5GHz 채널 중에서 빈 채널을 찾아 쓰면 된다. 비용 우선이면 Opal을 고려하되 펌웨어의 채널 제어 기능을 확인해야 하고, 균형을 맞추려면 구형 Beryl을 중고나 세일로 노려보는 게 낫다. 정확한 현재 가격과 스펙은 변동하니 직접 확인하는 게 맞다. 그럼 현행 저가 5GHz 모델은 Opal이 주력인 것 같다. 검색으로 정확한 스펙과 가격을 확인해야겠는데, 핵심은 이미 명확하다: 5GHz가 필수라면 2.4GHz만 지원하는 초저가 모델은 제외되고, GL.iNet의 저가 5GHz 옵션은 Opal이나 구형 Beryl 정도인데, 채널 제어를 위해서는 OpenWrt 지원이 있으면 좋다.

사용자가 구체적인 모델과 대략적인 가격을 물었으니 검색으로 현행 정보를 확인해야겠다.
```
This block is not supported on your current device yet.
```

Opal GL-SFT1200 확인 중인데, 듀얼밴드 AC1200으로 5GHz를 지원하고 USB-C 5V/3A 전원을 사용해서 Beryl AX와 동일한 전원 설정을 그대로 재사용할 수 있겠다. OpenWrt 기반이라는 것도 확인했다.

가격도 훨씬 저렴하고(보통 $30-40 vs Beryl AX $80~) 핵심 요구사항들을 모두 충족한다. WiFi 6를 포기하고 WiFi 5(ac)로 내려가지만 5Mbps 영상에는 충분하고, 더 큰 문제는 DFS 지원이 제한적일 수 있다는 점이다. AC1200 대역폭이라 DFS 채널을 지원하지 않거나 제한적이어서 다른 팀들과 같은 일반 5GHz 채널(36-48, 149-165)에서 경쟁해야 하고, 혼잡 회피 카드 하나를 잃게 된다. 그래도 5GHz 유지, USB-C 전원, OpenWrt 채널 제어라는 핵심은 모두 갖춘 솔루션이다. Opal이 최선인데, WiFi6와 DFS는 포기하되 일반 5GHz 채널 잘 고르면 충분하다. 절대 2.4GHz만 쓰는 저가형으로 내려가면 안 되는데, 대회장 혼잡에서 망가진다 — 5GHz는 필수다.있다. 같은 GL.iNet의 **Opal (GL-SFT1200)**이 딱 맞다. Beryl AX보다 한참 싸고(대략 절반 이하), 양보 못 하는 건 다 갖췄다.

- 듀얼밴드 AC1200, 300Mbps(2.4GHz) + 867Mbps(5GHz) — 5GHz 유지. 대회장 혼잡 회피의 핵심.
- 전원 입력 USB-C 5V/3A — Beryl AX랑 동일하니 짜둔 벅+필터 그대로 쓰면 된다.
- OpenWrt 기반 — 채널 수동 고정 자유도가 살아있다. 빈 채널 골라 박는 게 가능하다는 뜻이라 중요하다.

포기하는 건 두 가지다. WiFi 6 → WiFi 5(ac)인데, 영상 5Mbps엔 867Mbps도 차고 넘쳐서 무관하다. 진짜 아쉬운 건 DFS다 — Opal은 DFS 채널 지원이 약해서, 다른 팀들과 같은 일반 5GHz 대역(UNII-1/3)에서 경쟁해야 한다. Beryl AX였으면 비어있는 DFS 대역으로 피신할 카드가 있었는데 그게 없어진다. 그래도 일반 5GHz에서 빈 채널을 잘 스캔해 고르면 대부분 커버된다.

마지노선 하나. 더 싸게 가겠다고 2.4GHz only 모델(Mango GL-MT300N 같은 초저가)로는 절대 내려가지 마라. 대회장 2.4GHz는 사실상 죽은 대역이라 그건 돈 버리는 거다. 5GHz가 양보 불가다.

정리하면 Opal이 가성비 답이다. DFS 회피 카드를 포기하는 대신 값을 아끼는 트레이드오프인데, 채널 스캔만 부지런히 하면 실전에서 큰 차이 안 난다.

### User · 2026-06-17T18:43:17.465823Z

일단 필요 스펙 보수적으로 산정해줘

### Claude · 2026-06-17T18:44:34.127634Z

사용자가 라우터, 전원, 네트워크 스펙을 보수적으로 산정해달라고 요청했다. 지금까지 논의한 라우터 모델(Opal 또는 Beryl AX), USB-C 벅 컨버터 전원, 영상과 좌표 데이터를 위한 WiFi 대역폭 등을 정리하고 여유 있게 재계산해야 한다.

보수적 산정의 의미는 최악의 경우를 기준으로 여유 마진을 두는 것이다. D435i 카메라의 720p 30fps 영상 스트림을 기준으로 x264 저지연 인코딩 시 평균 4Mbps, 피크 6~8Mbps 정도로 잡고, 좌표 메타데이터 대역폭, 라우터 전원 요구사항(W, A), 벅 컨버터 입출력 스펙, 필터 정격 등을 각각 여유 있게 산정해야 한다.

YOLO 바운딩박스와 XYZ 좌표 메타데이터는 프레임당 몇 KB 수준이라 30fps 기준 초당 수백 KB 정도로 1Mbps 미만이고, 원격주행 제어 명령은 0.1Mbps 미만이다. 따라서 총 WiFi 처리량은 영상 피크 8Mbps에 메타데이터와 제어 신호를 더해 보수적으로 산정하면 된다.

애플리케이션 레이어에서 약 10Mbps가 필요한데, WiFi 실효 처리량이 PHY rate의 40-50% 수준이므로 안정적으로 50Mbps 이상의 PHY rate가 필요하다. Opal의 5GHz 867Mbps 스펙은 대회장 혼잡에서 크게 감소하더라도 10Mbps 애플리케이션 요구사항을 충분히 만족할 수 있으므로 대역폭 측면에서는 문제없다.

실제 병목은 대역폭이 아니라 혼잡, 손실, 지연이다. 라우터 전원 공급은 USB-C 5V/3A 스펙으로 최대 15W를 제공할 수 있으며, 실제 소비는 5GHz 풀로드 송신 시 5-8W 정도이므로 부팅 순간 전류 급증을 고려해도 충분한 여유가 있다.

벅 컨버터는 iEA의 18V 출력을 입력으로 받는데, 회생 시 전압 변동이 있어도 9-36V 범위를 지원하는 모듈이면 안정적이다. 출력은 연속 5V/3A를 보장해야 하므로 보수적으로 5A급 모듈을 선택하고, 입력 전류는 약 0.93A 정도로 작아서 문제없다.

필터와 커넥터는 라우터의 3A 소비에 마진을 더해 5A급 부품으로 정격을 잡고, 캐패시터는 회생과 스파이크를 고려해 50V 이상으로 선택한다. USB-C 케이블은 3A 표기 제품을 쓰고, 이더넷은 Jetson과 라우터 사이를 Cat5e/6로 짧게 연결하면 된다.

대역폭은 실제로 병목이 아니라는 점이 중요한데, 보수적으로 잡아도 10Mbps 수준 정도면 충분하다. 720p30 컬러를 기본 가정으로 진행하되 사용자가 다른 의도면 조정할 수 있도록 명시하겠다. 스펙 산정은 항목이 많고 숫자 중심이라 마크다운 표가 가장 명확한데, 모바일에서 깨질 수 있으니 텍스트 리스트나 간단한 표 형식으로 구성하는 게 낫겠다.

보수적 산정이므로 필요값과 권장 스펙(마진 포함)을 구분해서 제시하고, 각 항목마다 근거를 함께 설명하겠다. 영상 비트레이트는 평균 4Mbps, 피크 8Mbps로 잡되 험지와 고움직임 스파이크를 고려해서 상한을 설정하겠다.

좌표와 제어 신호는 1Mbps 미만, 애플리케이션 전체로는 약 10Mbps 보수 산정. WiFi는 5GHz 대역에서 실효 처리량이 PHY 스펙의 절반 이하이고 혼잡을 감안하면, 10Mbps 애플리케이션에는 어떤 5GHz 라우터도 충분하므로 대역폭은 병목이 아니고 실제 제약은 지연과 손실이다. SRT 레이턴시는 100~150ms 버퍼로 관리하고, 라우터 전원은 5V/3A(15W) 전량 공급에 인러시 여유를 더해서 벅 컨버터는 5V/5A급으로 선정하며, 입력은 9~36V(18V 기준) 범위에서 50V 이상 정격으로 보호회로를 갖춘 제품을 고르겠다.

회생 스파이크를 다시 생각해보니, 벅 입력은 iEA 출력인 18V이지 48V가 아니다. iEA가 48V를 18V로 절연 강압하므로 벅 입력 18V는 비교적 안정적이고, 회생 스파이크는 iEA 입력 48V 라인에서 발생하지만 iEA가 이를 18V로 절연하기 때문에 벅 입력 쪽은 영향이 제한적이다. 벅 입력범위 9~36V면 18V 변동을 충분히 흡수할 수 있으므로 50V 정격은 과할 수도 있고, 18V 라인이니 36V 정격 정도면 충분할 것 같다.

필터와 케이블 사양을 정리하면, CMC와 페라이트는 5A 이상, 캡은 50V 이상, USB-C는 3A 인증, 이더넷은 Cat5e 짧게 구성하면 된다. 대역폭 측면에서는 보수적으로 잡아도 10Mbps 이상이고 5GHz는 수백Mbps급이므로 충분한 여유가 있다.

핵심은 대역폭 걱정보다 전원 마진과 지연/손실 예산에 집중하는 것. 720p30 기준으로 보수적 상한으로 산정했으니, 해상도나 프레임레이트가 다르면 조정하면 된다.전제: color 720p30 단독 스트리밍 + 좌표 분리, x264 SW 인코딩 기준으로 잡는다. 다른 해상도 쓸 거면 영상 항목만 조정하면 된다.

**대역폭**
- 영상: 720p30 x264 zerolatency면 평균 ~4Mbps, 험지 고움직임 spike까지 보수적으로 **peak 8Mbps**.
- 좌표 JSON + 조종 입력: 합쳐서 **1Mbps 미만**, 보수적으로 1Mbps.
- 애플리케이션 총합 ≈ **10Mbps**.

여기서 중요한 결론. 이 10Mbps는 Opal의 5GHz(867Mbps PHY) 기준으로 차고 넘친다. 혼잡으로 실효 throughput이 90% 깎여도 여유다. **즉 대역폭은 병목이 아니고, 라우터 "속도" 스펙은 신경 안 써도 된다.** 진짜 산정 대상은 지연·손실 예산(SRT latency 100~150ms 버퍼로 흡수)과 아래 전원 쪽이다.

**라우터 전원**
- Opal/Beryl AX 정격 5V/3A = **15W**. 실소비는 5GHz 풀TX에서 5~8W지만, 부팅 inrush + 마진 잡아 15W 전량 공급 가능하게 설계.

**벅 컨버터**
- 입력: 18V 도메인(iEA 출력). 입력범위 **9~36V** 모듈이면 18V에 2배 마진.
- 출력: 5V 연속, 라우터 3A의 1.5배 마진 잡아 **5A급**.
- 입력전류: 15W ÷ 18V ÷ 0.9 ≈ **1A 미만**.
- 보호: 과전류/과열/단락 내장.

**필터 부품 정격**
- 전류: 라우터 3A + 마진 → **5A급** (CMC, 페라이트 비드).
- 전압: 18V 라인이지만 회생 변동 고려 캡 정격 **50V** (흔하고 여유 충분).

**케이블**
- USB-C: 3A 인증 케이블, 짧게.
- 이더넷: Jetson↔라우터 Cat5e 이상, 짧게.

요약하면 보수적으로 잡아도 대역폭은 남아돌고, 산정의 실질은 전원 15W 공급 마진 + 벅/필터 5A·50V 정격 + 지연 버퍼다. 영상 해상도를 720p보다 올릴 계획이면 그때 비트레이트만 다시 잡으면 된다.

### User · 2026-06-25T19:15:56.248352Z

라즈베리파이용 lcd 디스플레이 젯슨에 연결 못하냐

### Claude · 2026-06-25T19:16:22.357885Z

할 수 있다. 다만 "라즈베리파이용"이라는 한 단어 안에 호환되는 것과 거의 안 되는 게 섞여 있어서, 인터페이스부터 갈라야 한다.

HDMI 디스플레이 — 된다. Jetson Orin Nano에 HDMI(또는 DP) 출력이 있으니, HDMI로 받는 Pi용 LCD는 그냥 모니터로 붙는다. 전원만 따로 주면 끝이고 Pi 의존성 없다. 제일 안전한 선택.

DSI 디스플레이(Pi 공식 7" 터치 등) — 사실상 안 된다. DSI 커넥터가 물리적으로 있어도 핀 배열·드라이버가 Pi 전용이라 그대로는 안 뜬다. 건드리지 마라.

SPI 소형 LCD(ILI9341 등 흔한 1.5~3.5" 모듈) — 전기적으로는 연결되는데, 이런 모듈은 Pi의 `fbtft`/DT overlay에 의존해서 Jetson에선 그게 안 먹는다. SPI로 직접 프레임 그리는 라이브러리(luma.lcd, Adafruit CircuitPython displayio 등)로 화면을 코드로 직접 쳐야 동작한다. 작은 텍스트/상태표시면 쓸 만하지만 영상은 무리.

HDMI 입력 받는 3.5~7" Pi LCD(드라이버보드+HDMI) — 된다. 이건 사실상 작은 HDMI 모니터라 SPI 터치만 빼면 화면은 바로 뜬다.

근데 용도를 짚고 싶다. 이 LCD를 로봇에 달아 영상 스트림을 보려는 거면 방향이 틀렸다 — 영상은 라우터 거쳐 노트북에서 보는 게 이번 설계 전체의 골자고, Jetson은 헤드리스로 굴리는 게 맞다. LCD는 로봇 옆에서 IP·전압·CAN 상태·온도 같은 **로컬 상태 표시**용으론 유용하다.

뭘 띄우려는 거야 — 영상이야 상태값이야? 그거에 따라 추천이 갈린다.

### User · 2026-06-25T19:18:09.778510Z

DSI 디스플레이(Pi 공식 7" 터치) 이거 달려고 했었는데.

### Claude · 2026-06-25T19:18:24.746459Z

그거였으면 멈춰서 다행이다. Pi 공식 7" DSI는 Jetson Orin Nano에 사실상 안 붙는다. Jetson 캐리어보드에 DSI 커넥터가 물리적으로 있더라도, Pi 공식 디스플레이는 자체 브리지 보드 + Pi 전용 DSI 타이밍/드라이버에 묶여 있어서 핀맵도 드라이버 지원도 Jetson 쪽엔 없다. 커넥터 모양이 같아 보여도 그대로 꽂으면 안 뜨고, DSI는 핀 잘못 매칭하면 태울 수도 있어서 무턱대고 시도하면 위험하다.

원하는 게 7" 터치스크린이면, 같은 폼팩터로 **HDMI 입력 + USB 터치** 버전을 사라. Waveshare 같은 데서 7" IPS를 HDMI 받고 터치는 USB로 빼주는 모델을 판다. 이건 Jetson에선 그냥 모니터 + USB 마우스로 인식돼서 별도 드라이버 없이 바로 된다. DSI 버전과 가격도 비슷하다.

다만 용도를 다시 확인하고 싶다. 이 터치스크린을 로봇에 달려는 거면, 영상 보기용은 아니다 — 영상은 노트북에서 보는 게 이번 설계 전체의 골자니까. 로봇에 붙은 터치 7"는 보통 현장에서 모드 전환·캘리브레이션·IP/상태 확인 같은 **로컬 조작 패널**로 쓴다.

터치로 뭘 하려던 거야? 로컬 조작 패널이면 HDMI+USB 7"로 바로 가면 되고, 혹시 조종스테이션 쪽에서 쓸 거면 또 얘기가 다르다.

### User · 2026-06-25T19:19:24.536779Z

그냥 상태 모니터링 정도로 쓰려 했지. 터치는 굳이 필요 없어.

### Claude · 2026-06-25T19:20:09.216480Z

사용자가 터치 입력이 필요 없고 순수하게 상태 모니터링만 필요하다는 점이 명확해졌으니, 디스플레이 선택이 간단해진다. IP, 전압, CAN 상태, 온도, 모터 상태 같은 텍스트와 숫자 기반 정보를 표시하면 되므로 터치 기능은 불필요하다. 작은 HDMI LCD 패널을 Jetson에 직접 연결하는 방식이 가장 간단한 옵션이 될 것 같다.

다만 Jetson Orin Nano를 헤드리스로 운영하기로 했다면, HDMI 디스플레이를 붙일 때 데스크톱 환경을 띄우는 것이 리소스를 많이 먹을 수 있다. 대신 I2C OLED 같은 작은 디스플레이를 GPIO로 직접 연결하고 Python 라이브러리로 상태 정보를 그려내는 방식이 더 가볍고 전력 효율적일 것 같다. 정보량이 적으면 0.96인치나 1.3인치 OLED 패널로 충분하고, 더 많은 정보나 그래프가 필요하면 작은 HDMI LCD를 고려할 수 있다.

실제로는 물리 디스플레이를 달지 않고 같은 네트워크에 있는 노트북 브라우저에서 웹 대시보드로 상태를 확인하는 게 가장 깔끔할 수 있다. Jetson에 Flask 같은 간단한 웹서버를 띄워서 상태 JSON을 제공하면 추가 부품이나 배선 없이 어디서든 확인할 수 있다. 다만 사용자가 로봇 옆에서 물리적으로 상태를 보고 싶을 수도 있다는 점을 고려해야 한다.

물리 디스플레이가 필요하다면 I2C OLED가 최선이다. SSD1306이나 SH1106 같은 작은 OLED 화면을 I2C로 연결하면 전선은 4개뿐이고, luma.oled 라이브러리로 IP, 전압, 온도, CAN 상태, FPS 같은 정보를 몇 줄로 표시할 수 있다. DSI나 HDMI LCD보다 훨씬 간단하고 전력도 거의 안 쓴다.

더 많은 정보를 보고 싶거나 이미 노트북이 같은 네트워크에 있다면 다른 방식을 고려할 수 있지만, 현장에서 빠르게 상태를 확인하는 게 목표라면 I2C OLED로 충분하다.

사용자가 처음부터 물리 화면을 원했던 맥락을 보면 OLED가 최선의 선택이고, 헤드리스 Jetson 환경에서는 GPIO/I2C 기반 OLED가 HDMI 큰 디스플레이보다 훨씬 실용적이다. 또는 Flask 웹 대시보드로 노트북이나 폰에서 확인하는 방법도 있지만, 현장에서 기기 없이 로봇만 보고 상태를 즉시 확인해야 한다면 OLED가 더 실용적이다.터치 빼고 상태 표시뿐이면 Pi LCD(HDMI든 DSI든)는 과하다. 헤드리스로 굴릴 Jetson에 HDMI LCD를 붙이면 화면 채우려고 데스크탑 GUI를 띄워야 하는데, 그게 상태 몇 줄 보자고 리소스 먹는 짓이다.

그 용도엔 **I2C OLED**가 정답이다. SSD1306 0.96"나 SH1106 1.3" 같은 거. 이유:

- Jetson 40핀 헤더에서 4선(3.3V·GND·SDA·SCL)이면 끝. 별도 전원 없음.
- `luma.oled` 파이썬으로 IP·전압·온도·CAN ok·fps를 직접 텍스트로 찍으면 된다. 데스크탑 불필요 = 헤드리스에 딱 맞음.
- 전력·무게 무시 수준이라 로봇에 붙이기 부담 없다.

글자 더 크게/컬러로 원하면 SPI ILI9341 2.4~2.8"로 올리면 되는데, 이것도 `luma`나 직접 SPI로 그리는 거지 Pi의 fbtft에 의존하면 안 된다(Jetson에선 안 먹는다).

대안 하나. 어차피 라우터 같은 망에 다 있으니, 물리 화면 없이 Jetson에 Flask 상태 페이지 하나 띄워서 노트북/폰 브라우저로 봐도 된다. 부품 0이고 정보량 제약도 없다. 다만 대회장에서 노트북 안 켜고 로봇만 슥 봐서 IP·전압 확인하고 싶은 거라면, 그땐 OLED가 낫다.

현장에서 로봇 보면서 직접 확인하려는 거지? 그럼 I2C OLED로 가면 된다.

### User · 2026-06-25T19:25:17.071104Z

i2c 작은거 말고 제대로된 dp 7인치짜리 디스플레이 단다 치면 어떻게 활용할까

### Claude · 2026-06-25T19:26:19.053579Z

사용자가 OLED 대신 제대로 된 7인치 DP 디스플레이를 장착하는 걸로 방향을 바꿨고, Jetson Orin Nano의 DisplayPort 출력을 활용해서 상태 모니터링 화면을 구성하려고 생각 중이다.

이제 풀 화면 7인치면 OLED보다 훨씬 더 많은 정보를 표시할 수 있다. 헤드리스 구성이 아니라 GUI를 띄워야 하는데, 무거운 데스크탑 환경 대신 키오스크 모드로 단일 앱만 실행하는 방식으로 가면 된다. 실시간 대시보드로 카메라 프리뷰와 YOLO 감지 박스를 표시하고, CAN 노드들(ODrive, AK45, Dynamixel)의 전압/전류/온도, 프레임레이트, 스트림 비트레이트, 라우터 신호 강도, SRT 연결 상태 같은 각 노드 상태를 한눈에 볼 수 있게 구성할 수 있다. 대회 규정상 조종자가 로봇 화면을 확인하면서 인식 결과를 작성해야 하는데, 이 디스플레이가 그 용도로 딱 맞을 것 같다.

로봇 탑재 화면은 조종스테이션 화면과 다르게, 경기장 안의 팀원들이 로봇 상태를 즉시 확인하고 어느 노드가 문제인지 판단하는 데 쓸 수 있다. 리허설이나 세팅 중에 노트북 없이 로봇만으로 캘리브레이션과 디버깅을 할 수 있다는 게 큰 장점이다. 또한 노트북의 WiFi 스트림이 끊겨도 로봇 화면에는 항상 raw 카메라 영상이 표시되니 백업 역할도 한다.

기술적으로는 Jetson에 X나 Wayland 같은 디스플레이 서버를 띄워야 하는데, 풀 데스크탑은 너무 무거우니 kiosk 모드나 가벼운 대시보드(matplotlib, PyQt, 또는 로컬 Flask 페이지를 크로미움 kiosk로 띄우기) 같은 옵션을 고려 중이다.

영상 프리뷰는 GPU 가속(GStreamer 같은 도구)으로 처리하면 CPU 부하를 줄일 수 있고, Orin Nano의 NVDEC 하드웨어 디코더를 활용할 수 있다. 다만 RealSense 카메라는 raw 프레임을 바로 표시하면 되니 인코딩/디코딩이 필요 없다.

리소스 경합이 문제인데, 대시보드와 카메라 프리뷰를 Jetson에서 렌더링하면서 동시에 YOLO 추론을 돌리기가 까다로울 수 있다.

사용자가 원래 원한 건 상태 모니터링 정도였는데, 7인치 화면을 달면서 더 활용할 수 있는 게 뭐가 있을까 궁금해하는 것 같다. 솔직하게 말하면 7인치는 OLED보다 정보를 더 많이 표시할 수 있지만, 헤드리스 방식의 리소스 절약 장점을 포기하는 트레이드오프가 있다. 그리고 로봇은 경기 중에 사람이 만질 수 없으니(규정), 7인치의 실질적 가치는 리허설이나 세팅 단계에서 주로 나온다.

구체적으로는 풀스크린 대시보드로 노드 상태, 전압, 온도, fps, 링크 같은 정보를 한눈에 보거나, 카메라 원본 영상과 YOLO 감지 결과를 독립적으로 미리보기해서 송신 파이프라인과 상관없이 디버깅할 수 있다. 경기장에서 팀원이 로봇 옆에서 즉시 상태를 파악할 수 있고, 노트북 없이도 세팅과 정비를 단독으로 진행할 수 있다는 게 핵심이다. 솔직하게 경기 중에는 직접 조작할 수 없다는 한계와 리소스 경합 문제를 명시해야겠다. Jetson Orin Nano의 공식 개발 키트는 DP 포트를 지원하니 7인치 디스플레이 연결은 가능하다고 전제하고, 핵심은 무거운 데스크탑 환경 대신 단일 앱 키오스크 모드로 헤드리스 정신을 유지하면서 화면을 활용하는 것이다.화면을 키운 만큼 OLED로는 못 하던 게 열린다. 다만 무거운 데스크탑(GNOME 등)을 띄우는 순간 Orin Nano 리소스를 YOLO랑 나눠 먹으니, **풀 데스크탑 말고 kiosk 모드로 단일 앱만 풀스크린**으로 띄우는 게 핵심 전제다. 경량 WM(openbox) + 대시보드 앱 하나, 영상은 GStreamer GPU sink로. 그래야 헤드리스의 리소스 이점을 거의 유지하면서 화면을 쓴다.

활용은 셋으로 갈린다.

**1. raw 카메라 + YOLO 프리뷰 (제일 값짐)**
송신 파이프라인과 완전히 독립된, 로봇 로컬의 깨끗한 미리보기다. D435i color frame에 YOLO 박스랑 XYZ를 바로 찍어서 띄운다. 노트북 스트림이 WiFi로 끊기든 인코딩이 문제든, 이 화면은 항상 나온다. 그래서 **"카메라가 뭘 보고 있나 / YOLO가 제대로 잡나"를 로봇 옆에서 즉시 확인**하는 데 최강이다. 스트림 품질 디버깅할 때 송신측이 멀쩡한지 여기서 바로 판단된다.

**2. 통합 상태 대시보드**
OLED엔 안 들어가던 걸 다 띄운다 — CAN 10노드 생사, 셀/레일 전압, 모터 온도, 추론 fps, SRT 링크 상태와 RSSI, 현재 구간·자율/원격 모드. 경기장 안 팀원이 정비·트랙복귀 때 어느 노드가 죽었는지 슥 보고 판단하는 용도로 실전성이 있다.

**3. 노트북 없는 단독 운용**
리허설·세팅·캘리브레이션을 로봇 화면만으로 끝낸다. 부스에서 노트북 안 펴고 빠르게 점검할 때 편하다.

솔직한 한계도 짚자. 경기 중엔 규정상 조종자는 스테이션 안, 경기장엔 팀원 2명만 들어가고 로봇을 함부로 못 만진다. 그래서 7인치의 실시간 효용은 제한적이고, **진짜 가치는 리허설·정비·디버깅 단계**에 있다. 실시간 조종 영상은 어차피 노트북이 본다. 그 점만 인정하면 7인치는 OLED보다 분명히 쓸모가 크다 — 특히 1번, 파이프라인 독립 프리뷰가 세팅 기간 내내 시간을 아껴준다.

대시보드를 Qt로 짤지 로컬 웹페이지+크로미움 kiosk로 갈지 정하면, 경량 셋업 골격 잡아줄게.

### User · 2026-06-25T20:27:06.936594Z

그럼 터치기능까지 넣거나 다른 추가적인 input method가 있어야 활용 잘할수 있는거 아니냐

### Claude · 2026-06-25T20:27:29.647035Z

그렇지 않다. 화면은 보는 거고, 조작은 다른 데서 들어오면 된다. input을 화면에 붙이는 게 직관적으로 보이지만, 이 셋업에선 오히려 비효율이다.

이유는 둘이다. 첫째, 7인치를 의미 있게 쓰는 단계가 리허설·정비·디버깅인데, 그땐 거의 항상 Jetson에 SSH 붙은 노트북이 곁에 있다. 터미널·VS Code remote가 키보드라는 훨씬 강한 input이라, 화면 터치는 그 아래 단계 도구가 된다. 둘째, 경기 중엔 규정상 로봇을 함부로 못 만지니 로봇 위 터치는 어차피 못 쓴다.

그래서 input은 화면에 붙이지 말고 이렇게 분리하는 게 낫다.

- 주 input은 SSH. 무겁거나 정밀한 조작 전부 여기서.
- 화면은 출력 전용 대시보드. 단, **읽기 전용이 아니라 원격으로 조작되는 출력**으로 짜라. 로컬 웹 대시보드로 만들면 Jetson 화면엔 kiosk로 띄우고, 같은 페이지를 노트북·폰 브라우저로도 연다. 모드 전환·캘리브레이션 버튼을 거기 넣으면, 입력 하드웨어 0으로 폰에서 누르고 결과는 7인치로 확인하는 구조가 된다. 라우터로 다 같은 망이라 공짜다.
- 화면 단독으로 눌러야 할 게 진짜 있으면, 터치 추가하지 말고 GPIO 버튼 2~3개가 더 싸고 진동에 강하다. 모드 토글·리셋 같은 한두 동작이면 충분하다.

정리하면 input을 늘릴 게 아니라 위치를 옮기는 거다. 조작은 SSH랑 폰 브라우저, 7인치는 그 결과를 보는 출력. 이게 터치 다는 것보다 활용도가 높다.

터치를 굳이 고민한 게, 화면 보면서 그 자리에서 뭔가 누르고 싶은 특정 동작이 있어서야? 있으면 그게 GPIO 버튼감인지 폰 UI감인지 같이 갈라보자.

### User · 2026-06-25T20:28:27.669386Z

아니 로봇 본체에는 터치를 포함한 다른 입력장치 달기 싫어서, 그냥 디스플레이 하나만 달아도 활용도가 높은지 물어보고 싶었던거야

### Claude · 2026-06-25T20:28:38.367383Z

아 그거면 답은 명확하다. 디스플레이 하나만 달아도 활용도 충분히 높다. 입력장치 0개여도 손해 안 본다.

왜냐면 조작은 이미 SSH랑 폰/노트북 브라우저로 다 들어가니까, 본체엔 출력만 있으면 된다. 7인치는 그 결과를 보는 창이고, 입력을 본체에 붙일 이유가 처음부터 없었던 거다. 네가 본체에 입력장치 안 달고 싶다는 거랑 디스플레이만으로 잘 쓰는 거랑 전혀 안 부딪힌다.

입력 없는 순수 출력 화면으로도 값이 큰 게 앞서 말한 둘이다 — 파이프라인 독립 카메라+YOLO 프리뷰(스트림 끊겨도 항상 나오는 로컬 확인창), 그리고 통합 상태 대시보드(CAN·전압·온도·fps·링크). 둘 다 보기만 하면 되는 거라 터치가 필요 없다. 조작이 필요해지는 순간엔 폰으로 같은 웹 대시보드 열어서 누르면 되고, 그래서 본체는 깨끗하게 화면 하나로 끝난다.

정리하면 디스플레이만 다는 게 맞는 선택이고, 입력 안 다는 것도 맞다. 둘 다 옳다.

### User · 2026-06-25T20:29:38.585378Z

그럼 레포하고 우리 로봇 기능 다시 한번 본다음에 ui구성 어떤식으로 할지 한번 말해봐

### Claude · 2026-06-25T20:32:42.006509Z

사용자가 저장소와 로봇 기능을 다시 검토한 후 7인치 디스플레이 UI 구성을 제안해달라고 요청했다. 먼저 이전에 클론했던 저장소가 여전히 /home/claude/power-train-sw에 있는지 확인해야 하는데, 작업 간에는 파일시스템이 리셋되지만 같은 대화 내에서는 유지될 것으로 예상된다.

로봇의 전체 기능을 정리하면 Jetson Orin Nano를 중심으로 CAN 버스로 연결된 10개 노드(AK 모터, ODrive, Dynamixel)와 48V 배터리, RealSense 카메라, YOLO 기반 물체 인식, SRT 스트리밍을 통한 원격 제어 시스템이 있다. 대회 미션은 극한로봇과 국방로봇으로 나뉘며, 극한로봇은 험지 극복, 장애물 극복, 화재진압 지원, 재난구조 탐색 등의 기능을 수행해야 한다.

국방로봇은 계절별로 다른 미션을 수행하는데, 봄에는 안개 지역 정찰과 LED 기반 피아식별, 여름에는 구호물자 운반과 신호등 인식, 가을에는 비전마커 5개 식별, 겨울에는 빙판길과 제설 주행을 해야 한다. 정찰동행 모드에서는 선도로봇을 2m±0.5m 간격으로 추종하고, UI에서는 카메라 프리뷰와 YOLO 인식을 미션별로 다르게 표시해야 한다.

7인치 디스플레이를 단순 상태 대시보드가 아니라 미션 컨텍스트에 맞춰 동적으로 변하는 화면으로 설계하면, 정찰동행 간격 실시간 표시, 피아식별 결과, 비전마커 카운트, 구조요구자 인식(방번호/생존/사망), 신호등 상태 등 각 미션에 필요한 정보를 효과적으로 전달할 수 있다.

화면 전환은 자동 구간/모드 감지, 폰 웹 대시보드에서의 수동 전환, 또는 정보 밀도를 높여 한 화면에 통합하는 방식으로 구현할 수 있으며, 1280x800 해상도의 7인치 디스플레이를 기준으로 평상시/디버그 레이아웃과 미션별 레이아웃을 구분해서 설계하고 있다.

좌측에 카메라 피드와 YOLO 감지 박스, XYZ 좌표 오버레이를 배치하고 우측 사이드바에 배터리 상태(전압, 셀 정보, 전류, 잔량), CAN 노드 상태, 모터 온도, 추론 성능(FPS, CPU/GPU 사용률), 통신 링크 상태(SRT, RSSI, 비트레이트), 현재 모드와 구간 정보를 표시하는 구조로 가고 있다.

CAN 버스에 연결된 노드들을 정리하면 ODrive 3개, AK45 모터 4개, 그리고 추가 장치들이 있는데 정확한 구성이 불명확해서 일단 "CAN 노드 상태 그리드"로 일반화하기로 했다. 미션 런타임 레이아웃은 경기 중 보기 전용으로 영상과 인식 결과를 표시하되, 정찰동행 같은 특정 미션에서는 선도로봇과의 거리 게이지 같은 맞춤형 위젯을 추가할 예정이다.

미션 전환을 어떻게 처리할지가 문제인데, 자율주행 프로그램이 현재 단계를 알고 있으면 자동으로 전환하거나 폰에서 수동으로 전환하는 방식 중 하나를 택해야 한다. 현실적으로는 메인 대시보드 한 화면으로 통합하는 게 나을 것 같다.

YOLO가 감지하는 클래스에 따라 화면에 표시되는 위젯을 자동으로 결정하는 방식이 깔끔하다. 비전마커가 감지되면 마커 카운터를 띄우고, 사람이 감지되면 구조요구자 패널을 띄우는 식으로 YOLO 결과가 UI를 주도하게 하면 입력 없이도 자연스럽게 전환된다.

기술 스택으로는 로컬 웹 기반(브라우저 키오스크 모드)을 고려 중인데, 폰에서도 같은 페이지를 볼 수 있다는 장점이 있다. 레이아웃은 영상을 메인으로 하고 상태 정보를 사이드에 배치하는 구조로 가면 될 것 같다.

이제 데이터 수집 구조를 정리해야 하는데, 비전 노드(YOLO)에서 좌표를 메타데이터로 보내는 것처럼 CAN 버스(노드/온도/전압), 시스템 리소스(tegrastats), 링크 상태(SRT stats) 같은 데이터들을 한 곳에 모아서 UI가 구독하는 방식으로 가야 한다. 기존 파워트레인 소프트웨어가 ROS를 쓰는지 확인하고 그에 맞춰 pub/sub 구조를 설계해야 할 것 같다.

경기 중에는 화면을 볼 수 없다는 규정 때문에 런타임 미션 위젯의 실시간 효용이 제한되지만, 리허설과 디버그 단계에서 주로 쓰일 테니 UI는 디버그 친화적으로 설계하는 게 맞다. 일단 저장소 구조를 다시 확인해서 현재 데이터 흐름이 어떻게 되어 있는지 파악해야 한다.repo 다시 보고 정리하자.
```
This block is not supported on your current device yet.
```

구조를 보니 더 복잡하네. 드라이브 제어는 ODrive를 통해 BLDC 모터(BL70200)를 다루고, 스티어링은 AK 모터로 처리하는데 상태 확인과 캘리브레이션 로직이 포함되어 있다. 코너 모듈은 드라이브와 스티어링을 결합한 휠 모듈 형태고, 거기에 US-100 초음파 센서도 안전 시스템으로 통합되어 있다.

새로 파악한 부분은 이 로봇이 코너 모듈 기반 아키텍처를 쓰는 것 같다는 점이다. 각 코너에 ODrive로 구동하는 BLDC와 AK 모터로 조향하는 구조인데, 4개 코너면 모터 개수가 상당할 것 같다. 초음파 센서는 충돌이나 이탈을 감지해서 대회 감점을 피하는 안전 판정 시스템으로 작동하는 것 같고, 비전 시스템(YOLO, RealSense)과 Pi 서버, 노트북 클라이언트가 함께 동작하며 parameter_calc에서 JAX GPU를 써서 로봇 설계 파라미터를 최적화하고 있다. safety_monitor의 verdict 출력과 corner_module 상태 데이터 구조를 파악하는 게 핵심인데, 특히 충돌/이탈 경고를 UI에 크게 띄워서 정비팀이 즉시 인지하도록 하는 게 대회 감점 회피의 핵심이다. README와 corner_module, safety verdict 구조부터 빠르게 확인해야겠다.
```
This block is not supported on your current device yet.
```

로봇이 6륜 로커-보기 방식의 조향 로봇이고 각 코너에 조향 모터와 구동 모터가 있다는 걸 확인했다. 더 중요한 건 `motor_gui/` 폴더가 이미 존재하고 백엔드에 웹소켓과 asyncio 구현이 있다는 점인데, 이게 GUI 백엔드 개발이 이미 진행 중이라는 뜻이다.

코너 모듈 README에 명시된 미래 계획을 보니 각 CornerModule의 `state()` dict를 motor_gui가 텔레메트리로 노출하려는 설계가 이미 있다. 안전 센서(safety_us100)는 SAFE/WARN/STOP 3단계로 판정하고, STOP 상태에서 구동을 0으로 만들어 충돌을 방지한다. 코너 모듈은 arm/disarm/estop 상태 머신을 가지고 있다.

이제 motor_gui의 실제 구조를 살펴봐야 한다. 백엔드 서버와 프론트엔드가 어떻게 구성되어 있고 어떤 데이터를 노출하는지 확인해야 7인치 키오스크 UI 제안이 현실적이 될 수 있다.motor_gui가 이미 있네. 이걸 봐야겠다.
```
This block is not supported on your current device yet.
```

좋아, 이미 잘 구축된 motor_gui 시스템이 있네. FastAPI 백엔드가 WebSocket을 통해 텔레메트리를 스트리밍하고 REST API로 모터 제어, 설정 조회, 재연결, 녹화 기능을 제공하고 있다.

그렇다면 7인치 디스플레이는 Jetson에서 Chromium 키오스크 모드로 motor_gui 페이지를 띄우되, 터치 입력 없이 읽기 전용 대시보드로 사용하고 실제 조작은 노트북 브라우저에서 하는 게 맞다. 다만 motor_gui는 모터 튜닝 중심이라 현장 대시보드로는 비전 피드(YOLO/XYZ 프리뷰) 같은 다른 정보가 더 필요할 수 있다.

그래서 motor_gui의 WebSocket 텔레메트리 인프라와 capabilities 기반 동적 UI를 재사용하되, 비전 파이프라인, 안전 판정(SAFE/WARN/STOP), 시스템 헬스(전원/온도/CAN 노드 상태) 같은 빠진 소스들을 텔레메트리에 통합하는 방식으로 확장하면 된다.

7인치 디스플레이용으로는 같은 백엔드를 유지하되 프론트엔드 뷰만 달리해서, 진단용 플롯이 많은 화면 대신 한눈에 보는 대시보드 레이아웃으로 만들 계획이다.

ODrive의 vbus_voltage와 ibus를 활용하면 BMS 없이도 48V 레일 전압과 전류를 모니터링할 수 있고, 이미 모터 텔레메트리에 포함될 수 있다.

코너 모듈의 상태 정보도 텔레메트리로 노출할 예정이라, 6륜 로커-보기 로봇의 각 코너별 상태를 추적할 수 있게 된다.

이제 충분한 근거를 바탕으로 UI 구성을 제안할 수 있다. 로봇은 ZETIN 6륜 로커-보기 방위 로봇으로, 각 코너에 조향 AK40과 구동 ODrive(BL70200 BLDC)가 있고, 총 6개 BLDC 모터(ODrive 3개, 2축씩)와 4개의 AK 조향 모터를 CAN으로 제어한다.

센서와 입출력 구성으로는 US-100 초음파로 충돌 방지, D435i 카메라에 YOLO 비전 처리, DualSense 컨트롤러로 원격 주행, FastAPI 기반 모터 관제 GUI, SRT 스트리밍으로 영상과 좌표 데이터를 분리해 노트북으로 전송하고, 48V 전원과 라우터로 네트워크를 구성한다. 대회 미션은 피아식별, 비전마커 인식, 화재 타겟, 구조 요구자 상태 판별, 구호물품, 신호등, 선도로봇 추종 거리 등을 감지해야 한다.

7인치 디스플레이 UI는 기존 motor_gui의 WebSocket 텔레메트리 백본을 재사용하면서 현장 전용 뷰를 추가하고, Jetson에서 Chromium 키오스크로 풀스크린 표시한다. 출력은 이 디스플레이에서만 하고 조작은 노트북이나 폰의 motor_gui REST 인터페이스로 한다. 레이아웃은 1280x800 기준으로 좌측에 D435i 영상과 YOLO 박스, XYZ 좌표를 표시하되 스트림이 끊겨도 로컬 프리뷰가 유지되도록 하고, 미션 인식 대상을 자동으로 표시한다.

우상단에는 안전 신호등(SAFE 녹/WARN 황/STOP 적)과 US-100 거리 센서 값을 크게 표시해서 충돌이나 이탈 위험을 즉시 인지하고, 모드(자율/원격), arm/disarm/estop 상태, 현재 구간을 함께 보여준다. 우중단의 파워트레인 헬스 섹션은 6륜 로커 모식도에 각 코너의 상태를 색칠해서 표시하며, 각 코너별로 드라이브 회전 속도, 스티어 각도, 전류, 온도, 고장 상태를 나타내고 ODrive와 AK CAN 노드의 연결 상태도 표시한다. 우하단에는 시스템 전원 정보로 48V 레일 전압(ODrive vbus)과 배터리 상태를 표시한다.

텔레메트리 데이터는 motor_gui 워커에서 이미 ODrive와 AK 센서를 100Hz로 수집하고 있어서 vbus, 전류, 온도, 회전 수, 스티어 각도 등이 모두 나오고 있다. 비전 프로세스는 별도로 검출 결과와 XYZ 좌표를 같은 웹소켓이나 별도 채널로 전송하며, 안전 모니터 판정, 시스템 리소스 사용률(Jetson GPU/CPU/온도/추론 fps), 링크 상태(SRT, RSSI, 비트레이트, 패킷손실)를 모두 텔레메트리에 통합한다. 미션 위젯은 YOLO 클래스 기반으로 비전 마커 카운터, 피아 판정, 구조요구자 방별 결과, 신호등, 추종거리 게이지를 자동으로 표시한다.

경기 중에는 화면을 볼 수 없다는 규정이 있어서 실시간 효용은 리허설, 정비, 디버그에 집중하고, 안전 신호등과 코너 헬스, 비전 프리뷰가 정비팀이 트랙 복귀나 점검 때 빠르게 확인하는 용도로 가장 중요하다. 진단용 플롯은 노트북의 motor_gui에 남기고 7인치 화면은 요약 대시보드로 구성한다. 백엔드에서는 motor_gui 서버에 비전, 안전, 시스템, 링크 데이터를 합류시키고, 프론트엔드는 7인치 전용 뷰를 구현한다. 텍스트로 충분하고, 레이아웃은 ASCII 다이어그램이나 설명으로 표현하겠다.

아니면 7인치 대시보드 목업을 보여주는 게 더 직관적일까? 사용자가 실제로 어떻게 보일지 시각적으로 이해하기 쉬울 것 같은데. 텍스트로 명확하게 제안한 후 마지막에 "레이아웃 목업 그려줄까?" 정도로 물어보는 게 낫겠다. 이 단계에선 도구보다 텍스트 제안이 맞다.repo 다시 보니 핵심이 하나 있다. **이미 `motor_gui`라는 웹 관제 GUI를 만들어놨다.** FastAPI 백엔드가 WebSocket으로 100Hz 텔레메트리 쏘고 REST로 제어받고, 프론트는 바닐라 JS + uPlot에 capabilities 기반 동적 UI다. Jetson에서 돌리고 노트북 브라우저로 접속하는 구조 — 우리가 앞서 합의한 "로컬 웹 대시보드 + 폰/노트북 공유"랑 정확히 같다. 그러니 7인치 UI는 **새로 만들 게 아니라 motor_gui를 확장하고 거기에 전용 뷰 하나 얹는 것**이다.

로봇이 6륜 로커-보기에 코너 모듈(조향 AK40 + 구동 ODrive)이고, US-100 충돌방지가 SAFE/WARN/STOP verdict를 낸다는 것도 확인했다. 이게 UI 내용을 결정한다.

7인치를 4개 존으로 나누는 걸 제안한다(1280×800 기준).

**좌측 대형 — 비전 프리뷰.** D435i color + YOLO 박스 + XYZ를 로컬에서 직접 그린다. SRT 송신 파이프라인과 완전히 독립이라, WiFi로 노트북 스트림이 끊겨도 이 화면은 항상 나온다. "카메라가 뭘 보나 / YOLO가 제대로 잡나"를 로봇 옆에서 즉시 확인하는 창. 미션마다 인식 대상이 다르니(피아식별·비전마커·구조요구자·신호등·화재타겟) 잡힌 클래스가 그대로 뜨면 된다.

**우상단 — 안전 + 모드.** safety verdict를 대형 신호등으로(SAFE 녹/WARN 황/STOP 적) + 거리(mm). 충돌·이탈 임박을 정비팀이 즉시 인지 → 대회 감점 회피 도구다. 옆에 자율/원격 모드, arm/disarm/estop, 현재 구간.

**우중단 — 파워트레인 헬스.** 6륜 모식도에 코너별 상태를 색으로. 각 코너 구동 turns/s·조향°·전류·온도·fault, ODrive/AK CAN 노드 생사. corner_module의 `state()` dict가 이 소스다 — README에 "motor_gui 어댑터로 노출" 계획이 이미 적혀 있으니 방향이 맞다.

**우하단 — 시스템/전원/링크.** 48V 레일 전압과 버스 전류는 ODrive `vbus_voltage`/`ibus`에서 이미 텔레메트리로 들어온다(별도 BMS 안 읽어도 대략 파악). 거기에 tegrastats로 GPU/CPU·온도·추론 fps, SRT stats로 링크 상태·RSSI·패킷손실.

핵심은 데이터 합류다. motor_gui엔 모터 텔레메트리는 다 있는데 **vision·safety·system·link 네 소스가 빠져 있다.** WS가 JSON dict seam이라 확장이 쉬우니, 이 넷을 텔레메트리에 합류시키고 7인치 전용 프론트(`/field` 같은 뷰)가 구독하면 된다. 진단용 uPlot plot은 노트북 motor_gui에 그대로 두고, 7인치는 plot 없는 요약 대시보드로 가는 게 화면 크기·시인성에 맞다.

포커스는 명확히 하자. 경기 중엔 규정상 로봇을 못 만지니, 7인치의 실시간 효용은 **리허설·정비·디버그**에 있다. 정비팀이 트랙 복귀·점검 때 어느 코너가 죽었는지, safety가 왜 STOP 거는지, 카메라가 뭘 보는지 슥 보는 용도. 이 셋(safety 신호등 + 코너 헬스 + 비전 프리뷰)이 입력 0개로도 값이 큰 부분이다.

구현은 motor_gui 백엔드에 소스 4개 합류 → `/field` 뷰 추가 → Jetson에 chromium kiosk로 그 뷰 풀스크린, 이 순서다.

이 4존 레이아웃 목업으로 그려줄까? 아니면 텔레메트리 합류부터 스키마 잡을까 — 어디부터 갈지 골라줘.

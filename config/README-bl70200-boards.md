# BL70200 보드 레지스트리 (`config/bl70200_boards.json`)

USB 스키드 구성이 "어느 보드의 어느 축이 어느 바퀴인가"를 푸는 유일한 근거다.
CAN node 번호를 매개로 하므로 CAN 경로와 권위가 하나로 유지된다.

## 만드는 법 (젯슨, 컨테이너 안)

1. USB 에 붙은 보드 시리얼을 뽑는다.

   ```bash
   python3 -c "
   from corner_module.drive_odrive_usb_axis import UsbBoardPool
   print(UsbBoardPool().discover_serials())"
   ```

   ✅ 기대 출력: 시리얼 3개짜리 리스트.

2. 각 보드가 어느 node 쌍인지는 **CAN 셋업 때 정한 값**이다
   (`bl70200_setup.py --node N` 로 써 넣은 값). 보드 1장 = 연속한 두 node.

3. 아래 형식으로 저장한다. `[axis0_node, axis1_node]` 순서이며
   **axis1 = 로봇 우측**이므로 짝수 node 가 뒤에 온다.

   ```json
   {
     "<시리얼-1>": [11, 12],
     "<시리얼-2>": [13, 14],
     "<시리얼-3>": [15, 16]
   }
   ```

4. 검증한다.

   ```bash
   cd motor_control && python3 -c "
   from chassis.chassis_manager import build_usb_skid_corners
   c = build_usb_skid_corners('../config/bl70200_boards.json')
   print(sorted(c))"
   ```

   ✅ 기대 출력: 바퀴 6개 이름. ❌ `ValueError` 가 나면 메시지가 어느 node 가
   빠졌는지 / 미러 규약을 어겼는지 알려준다.

⚠️ 이 파일은 **기기마다 다르다**(시리얼이 보드 고유값). 레포에 커밋하지 않는다.

# Conveyor / Wheel Sorter 제어 — hwi_conveyor_test 참고본

이 파일은 `hwi_conveyor_test` 브랜치에서 사용하던 Conveyor / Wheel Sorter 제어 문서를 `hwi_new_sorter` 브랜치에 참고자료로 보존한 것이다.

> 중요: 이 문서의 `binary_switch`, `/World/ConveyorTrack/...` 경로, 단일 sorter 구조, 예전 속도 규칙은 현재 Parcel Sorting Map의 최종 기준이 아니다. 실제 구현은 `docs/conveyor_sorter_control.md`를 따른다.

## 과거 구조 요약

- 컨베이어는 `ConveyorBeltGraph`를 Stage에서 찾아 Python으로 Velocity를 설정했다.
- 예전 맵에서는 `/World/ConveyorTrack/ConveyorBeltGraph` 하나만 반대 부호로 제어했다.
- Wheel Sorter는 `/World/ConveyorTrack/Sorter/ActionGraph/binary_switch.inputs:value`를 True/False로 바꿨다.
- SorterSpeed는 `-1.0`을 사용했다.
- 테스트 박스는 `box_id` custom int attribute를 갖고 주기적으로 생성했다.
- 예전 `route_boxes()`는 박스 위치가 특정 X 임계값을 통과하면 `box_id`를 읽고 binary switch를 설정했다.
- 이 로직은 현재 맵의 `ConveyorTrack_01/_02/_03` 및 `conveyor_belt.inputs:direction` 구조와 다르므로 직접 실행용으로 사용하지 않는다.

## 참고 파일

```text
isaac_sim/equipment/conveyor/conveyor_controller_hwi_conveyor_test_reference.py
isaac_sim/equipment/wheel_sorter/wheel_sorter_controller_hwi_conveyor_test_reference.py
isaac_sim/cargo/box_spawner.py
```

실제 현재 구현 기준은 다음 문서를 사용한다.

```text
docs/conveyor_sorter_control.md
```

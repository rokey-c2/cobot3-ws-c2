# 생성형 AI 샷 프롬프트 — 애니메 사무실 남자 + 알림 카드

작성일: 2026-08-27 (시나리오 개정본) · 브랜치: `hwi_video_concept`
기반: `docs/video_editing_guide.md` 1장(60초 구성), 3-3 / 3-4 / 5장

---

## 0. AI 생성이 허용되는 범위

`video_ai_brief.md` 제약 기준:

| 요소 | AI 생성 | 비고 |
|---|---|---|
| 사무실 남자 플레이트 (줌 시퀀스 3번) | **허용** | 순수 서사·인물 연출. 애니메 풍이라 실사와 구분됨 |
| 알림 카드 문구 (4′) | AI에 맡기지 않음 | CapCut 텍스트 레이어로 얹음 (한글/정확도) |
| 로봇·창고·컨베이어·소터·**관제 대시보드** | **금지** | 실제 Isaac Sim / 실제 웹 녹화·스크린샷만 |

---

## 1. 툴

| 툴 | 종류 | 용도 |
|---|---|---|
| **Nano Banana / Nano Banana Pro** (Gemini 이미지) | **이미지** (비디오 X) | 사무실 남자 **스틸 플레이트** 생성 |
| CapCut 내장 AI (이미지→비디오) | 짧은 비디오 | 스틸에 미세 모션 부여 (선택) |
| Veo 3 / Kling / Luma | 텍스트/이미지→비디오 | 스틸에 움직임이 필요할 때 |

권장: **스틸 1~2장 생성 → CapCut에서 합성·모션**. 6~8초 한 컷에 고개 돌림까지 한 번에
생성하는 건 불안정하므로, 아래 2장처럼 "일하는 상태" / "고개 돌린 상태" 2장을 만들어
CapCut에서 전환한다.

---

## 2. 사무실 남자 플레이트 (줌 시퀀스 3번, 화면에 약 6초)

### 조건
- **애니메 / 셀 애니메이션 풍**, 차분한 톤
- **뒤에서 본 모습** (뒷통수·등이 보임, 얼굴 안 보임)
- **듀얼 모니터** 책상. 왼쪽 모니터를 보며 작업 중
- 오른쪽 모니터 화면은 **비어 있음**(단색/흐릿) — 관제 화면은 편집에서 얹음
- 카메라 고정. 이 플레이트 위로 편집에서 줌아웃/줌인이 지나감
- 16:9, 가능하면 1080p 이상

### 스틸 A — "왼쪽 보며 일하는 중" (Nano Banana / Nano Banana Pro)

```
Anime style, clean cel-shaded illustration, calm muted color palette.
Rear view of a man seated at a desk in a modern logistics operations
office, seen from behind so his face is not visible — back of his head,
shoulders and upper back fill the lower-center of the frame. A dual-monitor
setup on the desk: he is turned slightly toward the LEFT monitor, working,
one hand on the keyboard. The RIGHT monitor is on but shows only a plain
dim blue screen with no readable content. Soft daylight from a window,
subtle desk lamp. Locked eye-level camera, medium shot, 16:9. No text,
no captions, no watermark, no logos.
```

### 스틸 B — "오른쪽 모니터를 본다"

스틸 A 프롬프트에서 해당 문장을 교체:
`he has turned his head and upper body toward the RIGHT monitor, looking
at it; the left hand still rests near the keyboard.`

→ CapCut에서 A(0:20–0:24) → B(0:24–) 로 짧게 전환하거나, A→B 사이에 살짝
포지션/회전 키프레임으로 시선 이동을 만든다. (선택) Kling/Veo에 A를 넣고
`the man turns his head toward the right monitor, minimal motion, locked camera, 3s`.

### Negative / 피할 것
```
face visible, front view, photorealistic, 3D render, text on screen,
subtitles, watermark, logo, warped hands, extra fingers, multiple people,
fast motion, camera pan, camera zoom
```

---

## 3. 알림 카드 2장 (4′, 화면에 약 4초) — CapCut 텍스트 레이어

생성형 AI 아님. CapCut에서 직접:

| 카드 | 문구 | 스타일 |
|---|---|---|
| 1 | `Conveyor STOP` | 흰 카드(라운드 사각), 빨강 경고 아이콘 점, 텍스트 `#1F2937` Bold |
| 2 | `Urgent delivery 발생` | 같은 카드, `Urgent` 또는 전체 `#F59E0B`(주황) 강조, 한글은 본고딕/Pretendard Bold |

- 줌 시퀀스에서 사무실 플레이트 위에 **풀사이즈로 크게** 오버레이 (0:26–0:30)
- 카드1 슬라이드-인 → 0.3~0.5초 후 카드2 슬라이드-인 (위아래로 쌓기)
- 인과 매칭: `Conveyor STOP` → 이후 6b에서 컨베이어 재가동으로 해소. 별도 구역명/3중 매칭 없음

---

## 4. 출력 규격
- 16:9, 1080p 이상, 24~30fps
- 오디오 불필요
- 스틸은 넉넉히 여러 장 뽑아 CapCut에서 고름
- 워터마크 없는 내보내기 확인

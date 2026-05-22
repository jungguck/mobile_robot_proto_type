# 📑 LLM Wiki Schema & Conventions

이 문서는 이 프로젝트의 지식 베이스(Wiki)가 어떻게 관리되고 확장되는지에 대한 규칙을 정의합니다.

## 🏗️ 구조 (Structure)

- `docs/wiki/index.md`: 전체 페이지 목록 및 요약 (Content-oriented).
- `docs/wiki/log.md`: 모든 수정 및 분석 이력 (Chronological).
- `docs/wiki/entities/`: 로봇 하드웨어, 패키지, 특정 노드 등 객체 기반 페이지.
- `docs/wiki/concepts/`: TF, EKF, MPC, SLAM 등 이론 및 설계 철학 페이지.
- `docs/wiki/logs/`: 상세한 디버깅 로그 및 실험 데이터.

## 🔄 워크플로우 (Workflows)

### 1. Ingest (지식 흡수)
새로운 소스(코드, 문서, 사용자 요구사항)가 추가되면:
1. 소스를 정독하고 핵심 요점을 도출한다.
2. 관련 Entity 및 Concept 페이지를 생성하거나 업데이트한다.
3. `index.md`와 `log.md`를 갱신한다.
4. 새로운 지식이 기존 데이터와 충돌할 경우 이를 명시적으로 기록한다.

### 2. Query (질의 및 합성)
사용자가 질문을 하면:
1. `index.md`를 먼저 읽어 관련 페이지를 찾는다.
2. 해당 페이지들을 읽고 답변을 합성한다.
3. 합성된 답변 중 가치가 있는 내용은 새로운 위키 페이지로 생성한다.

### 3. Lint (건전성 검사)
주기적으로 다음 사항을 확인한다:
- 페이지 간의 모순(Contradictions).
- 오래된(Stale) 정보.
- 링크가 없는 고립된 페이지(Orphan pages).

## ✍️ 작성 스타일
- **Obsidain 친화적**: `[[Page_Name]]` 형태의 위키 링크 사용.
- **Citations**: 모든 주장은 소스 파일이나 로그를 인용한다.
- **Persistent & Compounding**: 단기 답변이 아닌, 누적되는 지식의 형태로 작성한다.

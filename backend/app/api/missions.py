from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field
from psycopg.rows import dict_row

from app.database import get_db_connection
from app.process_events import create_tracking_mission


router = APIRouter(
    prefix="/api/missions",
    tags=["missions"],
)


class MissionCreate(BaseModel):
    package_code: str = Field(min_length=1, max_length=100)
    region: str = Field(default="A", min_length=1, max_length=30)
    mission_code: str | None = Field(default=None, max_length=100)


@router.post("")
def create_mission(request: MissionCreate):
    """Create the active tracking mission used by live ROS2 events."""

    try:
        return create_tracking_mission(
            package_code=request.package_code,
            region=request.region,
            mission_code=request.mission_code,
        )
    except ValueError as error:
        raise HTTPException(status_code=409, detail=str(error))
    except Exception as error:
        raise HTTPException(
            status_code=500,
            detail=f"Failed to create mission: {error}",
        )


@router.get("/current")
def get_current_mission():
    """
    진행 중인 Mission을 우선 조회하고, 없으면 가장 최근 완료/실패 Mission과
    해당 Stage 목록을 반환한다.
    """

    try:
        with get_db_connection() as conn:
            with conn.cursor(row_factory=dict_row) as cursor:

                # Active Mission 우선, 없으면 가장 최근 종료 Mission 조회
                cursor.execute(
                    """
                    SELECT
                        id,
                        mission_code,
                        status,
                        current_stage,
                        started_at,
                        completed_at,
                        created_at
                    FROM mission
                    ORDER BY
                        CASE WHEN status IN ('READY', 'RUNNING', 'PAUSED')
                             THEN 0 ELSE 1 END,
                        created_at DESC
                    LIMIT 1;
                    """
                )

                mission = cursor.fetchone()

                if mission is None:
                    return {
                        "mission": None,
                        "stages": [],
                    }

                # 해당 Mission의 진행 단계 조회
                cursor.execute(
                    """
                    SELECT
                        id,
                        stage_code,
                        sequence_no,
                        status,
                        started_at,
                        completed_at
                    FROM mission_stage
                    WHERE mission_id = %s
                    ORDER BY sequence_no;
                    """,
                    (mission["id"],),
                )

                stages = cursor.fetchall()

        return {
            "mission": mission,
            "stages": stages,
        }

    except Exception as error:
        raise HTTPException(
            status_code=500,
            detail=f"Failed to load current mission: {error}",
        )

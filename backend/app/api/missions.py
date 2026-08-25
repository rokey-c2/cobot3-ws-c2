from fastapi import APIRouter, HTTPException
from psycopg.rows import dict_row

from app.database import get_db_connection


router = APIRouter(
    prefix="/api/missions",
    tags=["missions"],
)


@router.get("/current")
def get_current_mission():
    """
    현재 진행 중인 Mission 1개와
    해당 Mission의 Stage 목록을 조회한다.
    """

    try:
        with get_db_connection() as conn:
            with conn.cursor(row_factory=dict_row) as cursor:

                # 현재 진행 중인 Mission 1개 조회
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
                    WHERE status IN ('READY', 'RUNNING', 'PAUSED')
                    ORDER BY created_at DESC
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

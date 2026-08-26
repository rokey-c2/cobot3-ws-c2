from fastapi import APIRouter, HTTPException
from psycopg.rows import dict_row

from app.database import get_db_connection


router = APIRouter(
    prefix="/api/events",
    tags=["events"],
)


@router.get("")
def get_events():
    """
    Package Event Log를 최신순으로 조회한다.
    Control Tower Event Timeline에서 사용한다.
    """

    try:
        with get_db_connection() as conn:
            with conn.cursor(row_factory=dict_row) as cursor:
                cursor.execute(
                    """
                    SELECT
                        pe.id,

                        p.package_code,

                        z.zone_code,
                        z.name AS zone_name,

                        pe.event_type,
                        pe.result,
                        pe.occurred_at

                    FROM package_event pe

                    JOIN package p
                        ON p.id = pe.package_id

                    LEFT JOIN zone z
                        ON z.id = pe.zone_id

                    ORDER BY pe.occurred_at DESC

                    LIMIT 100;
                    """
                )

                events = cursor.fetchall()

        return {
            "count": len(events),
            "events": events,
        }

    except Exception as error:
        raise HTTPException(
            status_code=500,
            detail=f"Failed to load events: {error}",
        )

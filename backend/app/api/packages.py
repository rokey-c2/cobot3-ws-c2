from fastapi import APIRouter, HTTPException
from psycopg.rows import dict_row

from app.database import get_db_connection


router = APIRouter(
    prefix="/api/packages",
    tags=["packages"],
)


@router.get("")
def get_packages():
    """
    전체 Package의 현재 상태와 현재 Zone을 조회한다.
    """

    try:
        with get_db_connection() as conn:
            with conn.cursor(row_factory=dict_row) as cursor:
                cursor.execute(
                    """
                    SELECT
                        p.id,
                        p.package_code,
                        p.region,
                        p.status,

                        m.mission_code,

                        z.zone_code AS current_zone,
                        z.name AS current_zone_name,

                        p.created_at,
                        p.updated_at

                    FROM package p

                    LEFT JOIN mission m
                        ON m.id = p.mission_id

                    LEFT JOIN zone z
                        ON z.id = p.current_zone_id

                    ORDER BY p.id;
                    """
                )

                packages = cursor.fetchall()

        return {
            "count": len(packages),
            "packages": packages,
        }

    except Exception as error:
        raise HTTPException(
            status_code=500,
            detail=f"Failed to load packages: {error}",
        )


@router.get("/{package_code}")
def get_package(package_code: str):
    """
    Package 1개의 현재 상태와 Event History를 조회한다.
    """

    try:
        with get_db_connection() as conn:
            with conn.cursor(row_factory=dict_row) as cursor:

                cursor.execute(
                    """
                    SELECT
                        p.id,
                        p.package_code,
                        p.region,
                        p.status,

                        m.mission_code,

                        z.zone_code AS current_zone,
                        z.name AS current_zone_name,

                        p.created_at,
                        p.updated_at

                    FROM package p

                    LEFT JOIN mission m
                        ON m.id = p.mission_id

                    LEFT JOIN zone z
                        ON z.id = p.current_zone_id

                    WHERE p.package_code = %s;
                    """,
                    (package_code,),
                )

                package = cursor.fetchone()

                if package is None:
                    raise HTTPException(
                        status_code=404,
                        detail="Package not found",
                    )

                cursor.execute(
                    """
                    SELECT
                        pe.id,
                        z.zone_code,
                        z.name AS zone_name,
                        pe.event_type,
                        pe.result,
                        pe.occurred_at

                    FROM package_event pe

                    LEFT JOIN zone z
                        ON z.id = pe.zone_id

                    WHERE pe.package_id = %s

                    ORDER BY pe.occurred_at;
                    """,
                    (package["id"],),
                )

                events = cursor.fetchall()

        return {
            "package": package,
            "events": events,
        }

    except HTTPException:
        raise

    except Exception as error:
        raise HTTPException(
            status_code=500,
            detail=f"Failed to load package: {error}",
        )

from contextlib import asynccontextmanager

from fastapi import FastAPI, HTTPException

from app.database import get_db_connection
from app.mqtt_client import start_mqtt, stop_mqtt
from app.pose_mqtt import start_pose_mqtt, stop_pose_mqtt

from app.api.amr import router as amr_router
from app.api.equipment import router as equipment_router
from app.api.events import router as events_router
from app.api.manual import router as manual_router
from app.api.missions import router as missions_router
from app.api.packages import router as packages_router
from app.api.system import router as system_router


@asynccontextmanager
async def lifespan(app: FastAPI):
    # FastAPI 시작
    start_mqtt()
    start_pose_mqtt()

    yield

    # FastAPI 종료
    stop_pose_mqtt()
    stop_mqtt()


app = FastAPI(
    title="Control Tower API",
    version="0.1.0",
    lifespan=lifespan,
)


app.include_router(equipment_router)
app.include_router(missions_router)
app.include_router(packages_router)
app.include_router(events_router)
app.include_router(system_router)
app.include_router(amr_router)
app.include_router(manual_router)


@app.get("/")
def root():
    return {
        "service": "control-tower-backend",
        "status": "running",
    }


@app.get("/health")
def health():
    return {
        "status": "ok",
    }


@app.get("/health/db")
def database_health():
    try:
        with get_db_connection() as conn:
            with conn.cursor() as cursor:
                cursor.execute(
                    """
                    SELECT
                        current_database(),
                        current_user,
                        version();
                    """
                )

                database, user, version = cursor.fetchone()

        return {
            "status": "ok",
            "database": database,
            "user": user,
            "postgres_version": version,
        }

    except Exception as error:
        raise HTTPException(
            status_code=503,
            detail=f"Database connection failed: {error}",
        )

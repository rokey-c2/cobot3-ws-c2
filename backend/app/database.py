import os
from pathlib import Path

import psycopg


def get_db_connection():
    """
    PostgreSQL 연결을 생성한다.

    Docker 환경에서는 POSTGRES_HOST=postgres 로 설정한다.
    postgres는 compose.yaml의 PostgreSQL service 이름이다.
    """
    return psycopg.connect(
        host=os.getenv("POSTGRES_HOST", "localhost"),
        port=os.getenv("POSTGRES_PORT", "5432"),
        dbname=os.getenv("POSTGRES_DB", "control_tower"),
        user=os.getenv("POSTGRES_USER", "controltower"),
        password=os.getenv("POSTGRES_PASSWORD"),
        connect_timeout=5,
    )


def apply_migrations():
    """Apply idempotent SQL migrations on every backend startup."""

    migrations_dir = Path(__file__).resolve().parents[1] / "db" / "migrations"
    if not migrations_dir.is_dir():
        raise RuntimeError(f"Migration directory not found: {migrations_dir}")

    with get_db_connection() as conn:
        with conn.cursor() as cursor:
            for migration_path in sorted(migrations_dir.glob("*.sql")):
                cursor.execute(migration_path.read_text(encoding="utf-8"))
                print(f"[DB] Applied migration: {migration_path.name}", flush=True)
        conn.commit()

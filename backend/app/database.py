import os

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

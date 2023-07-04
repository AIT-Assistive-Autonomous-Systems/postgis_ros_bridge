# Copyright 2023 AIT - Austrian Institute of Technology GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Query class for executing SQL queries on the database."""
from contextlib import AbstractContextManager

from sqlalchemy import Result, text
from sqlalchemy.orm import sessionmaker

from postgis_ros_bridge.postgresql_connection import PostgreSQLConnection


class Query(AbstractContextManager):
    """Query class for executing SQL queries on the database."""

    def __init__(self, postgresql_conn: PostgreSQLConnection, query: str):
        session_ = sessionmaker(bind=postgresql_conn.engine)
        self._session = session_()
        self._query = text(query)

    def __enter__(self):
        return self

    def __exit__(self, _type, _value, _traceback):
        self._session.close()

    def get_results(self) -> Result:
        """Execute the query and return the results."""
        return self._session.execute(self._query)

    def __repr__(self) -> str:
        return super().__repr__() + f"({self._query})"

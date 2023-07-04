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

"""Module to transform a geodesic to a local cartesian frame."""

from typing import Tuple, SupportsFloat

from pyproj import Transformer, Proj
from pyproj.aoi import AreaOfInterest
from pyproj.crs import CRS
from pyproj.database import query_utm_crs_info

from geometry_msgs.msg import (Point, Point32)


class GeodesicTransform:
    """Transform geodesic coordinates into a local cartesian frame."""

    def __init__(self, crs_to, origin_transform: bool = False, origin_lon: float = 0.0,
                 origin_lat: float = 0.0) -> None:
        """Initialize GeodesicTransform."""
        self.origin_easting = None
        self.origin_northing = None
        self.origin_transform = False

        crs_from = {"proj": 'latlong', "ellps": 'WGS84', "datum": 'WGS84'}
        self.transformer = Transformer.from_crs(
            crs_from,
            crs_to
        )
        self.proj = Proj(self.transformer.target_crs)

        """Set map origin for local cartesian frame."""
        # pylint: disable=unpacking-non-sequence
        if origin_transform:
            easing, northing = self.transformer.transform(origin_lon, origin_lat)
            self.origin_easting = easing
            self.origin_northing = northing
            self.origin_transform = True

    @staticmethod
    def to_utm(zone: int, band: str, **kwargs) -> "GeodesicTransform":
        """Construct transfromer to UTM from fixed zone and band."""
        return GeodesicTransform({"proj": 'utm', "ellps": 'WGS84', "datum": 'WGS84', "zone": zone,
                                  "band": band}, **kwargs)

    @staticmethod
    def to_geocent(**kwargs) -> "GeodesicTransform":
        """Construct transfromer to geocentric model."""
        return GeodesicTransform({"proj": 'geocent', "ellps": 'WGS84', "datum": 'WGS84'}, *kwargs)

    @staticmethod
    def to_utm_lonlat(lon: float, lat: float, **kwargs) -> "GeodesicTransform":
        """Construct transfromer to UTM for given lat/lon."""
        utm_crs_list = query_utm_crs_info(
            datum_name="WGS84",
            area_of_interest=AreaOfInterest(
                west_lon_degree=lon,
                south_lat_degree=lat,
                east_lon_degree=lon,
                north_lat_degree=lat,
            ),
        )
        utm_crs = CRS.from_epsg(utm_crs_list[0].code)
        return GeodesicTransform(utm_crs, **kwargs, origin_lat=lat, origin_lon=lon)

    def transform_lonlat(self, lon: float, lat: float) -> Tuple[float, float]:
        """Transform a point from lat/lon to local map and optionally apply offset."""
        # pylint: disable=unpacking-non-sequence
        easting, northing = self.transformer.transform(lon, lat)
        if self.origin_transform:
            easting -= self.origin_easting
            northing -= self.origin_northing

        return easting, northing

    def meridian_convergence(self, lon: float, lat: float) -> float:
        """Return meridian convergence angle (rad) for given lon/lat."""
        factors = self.proj.get_factors(lon, lat)
        return factors.meridian_convergence

    def transform_point(self, point: Point) -> Point:
        """Transform a geodetic point to local cartesian frame."""
        easting, northing = self.transform_lonlat(lon=point.x, lat=point.y)

        return Point(x=easting, y=northing, z=point.z)

    def transform_point32(self, point: Point32) -> Point32:
        """Transform a geodetic point to local cartesian frame."""
        easting, northing = self.transform_lonlat(lon=point.x, lat=point.y)

        return Point32(x=easting, y=northing, z=point.z)

    def transform_tuple(
            self, point: Tuple[SupportsFloat,
                               SupportsFloat,
                               SupportsFloat]) -> Tuple[SupportsFloat,
                                                        SupportsFloat,
                                                        SupportsFloat]:
        """Transform a geodetic point to local cartesian frame."""
        easting, northing = self.transform_lonlat(lon=point[0], lat=point[1])

        return (easting, northing, point[2])

    def __repr__(self) -> str:
        """Return string representation of GeodesicTransform."""
        return f"{self.transformer}"

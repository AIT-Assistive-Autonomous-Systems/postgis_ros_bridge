-- Enable PostGIS (includes raster)
CREATE EXTENSION IF NOT EXISTS postgis;

-- Create the tables
CREATE TABLE public.point (
    id SERIAL PRIMARY KEY,
    geom GEOMETRY(PointZ, 4326)
);

CREATE TABLE public.linestring (
    id SERIAL PRIMARY KEY,
    geom GEOMETRY(LineStringZ, 4326)
);

CREATE TABLE public.polygon (
    id SERIAL PRIMARY KEY,
    geom GEOMETRY(PolygonZ, 4326)
);

-- Insert example data into the tables
INSERT INTO public.point (geom) VALUES 
    (ST_GeomFromText('POINT Z(0 0 0)', 4326)),
    (ST_GeomFromText('POINT Z(1 1 1)', 4326)),
    (ST_GeomFromText('POINT Z(2 2 2)', 4326)),
    (ST_GeomFromText('POINT Z(3 3 3)', 4326)),
    (ST_GeomFromText('POINT Z(4 4 4)', 4326));

INSERT INTO public.linestring (geom) VALUES 
    (ST_GeomFromText('LINESTRING Z(0 0 0, 1 1 1)', 4326)),
    (ST_GeomFromText('LINESTRING Z(2 2 2, 3 3 3)', 4326)),
    (ST_GeomFromText('LINESTRING Z(4 4 4, 5 5 5)', 4326)),
    (ST_GeomFromText('LINESTRING Z(6 6 6, 7 7 7)', 4326)),
    (ST_GeomFromText('LINESTRING Z(8 8 8, 9 9 9)', 4326));

INSERT INTO public.polygon (geom) VALUES 
    (ST_GeomFromText('POLYGON Z((0 0 0, 1 0 0, 1 1 0, 0 1 0, 0 0 0))', 4326)),
    (ST_GeomFromText('POLYGON Z((2 2 0, 3 2 0, 3 3 0, 2 3 0, 2 2 0))', 4326)),
    (ST_GeomFromText('POLYGON Z((4 4 0, 5 4 0, 5 5 0, 4 5 0, 4 4 0))', 4326)),
    (ST_GeomFromText('POLYGON Z((6 6 0, 7 6 0, 7 7 0, 6 7 0, 6 6 0))', 4326)),
    (ST_GeomFromText('POLYGON Z((8 8 0, 9 8 0, 9 9 0, 8 9 0, 8 8 0))', 4326));

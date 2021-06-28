import glob
import os

import geopandas as gpd
import pandas as pd
from gdal import ogr
from shapely.geometry import Polygon


def get_layers(ds):
    layer_count = ds.GetLayerCount()
    layers = []
    for i in range(layer_count):
        layer = ds.GetLayerByIndex(i)
        print(layer.GetName())
        layers.append(layer)
    return layers


def get_features(layer):
    feature = layer.GetNextFeature()
    features = []
    while feature is not None:
        features.append(feature)
        feature = layer.GetNextFeature()

    return features


def get_feature_info(feature):
    geom_ref = feature.GetGeometryRef()
    if geom_ref is not None and geom_ref.GetGeometryName() == "POINT":
        print(geom_ref.GetGeometryName())
        name_index = feature.GetFieldIndex("OBJNAM")
        name = "NO OBJNAM"
        if name_index != -1 and feature.GetFieldAsString(name_index) != "":
            name = feature.GetFieldAsString(name_index)
        feature_info = (name, feature.GetFID(), geom_ref.GetX(), geom_ref.GetY())
        # rospy.loginfo(featureInfo)
        if feature_info[2] != 0.0 and feature[3] != 0.0:
            print(feature_info)
            return feature_info


class Query:
    def __init__(self, enc_root="/home/thomasdonnelly/Downloads/ENC_ROOT"):
        filenames = glob.glob(os.path.join(enc_root, '*/*.000'))
        print("Filenames:" + str(filenames))
        feature_infos = []
        for filename in filenames:
            ds = ogr.Open(filename)
            layers = get_layers(ds)
            for layer in layers:
                features = get_features(layer)
                for feature in features:
                    feature_info = get_feature_info(feature)
                    if feature_info is not None:
                        feature_infos.append(feature_info)
        print(feature_infos)
        df = pd.DataFrame.from_records(feature_infos, columns=['name', 'fid', 'longitude', 'latitude'])
        self.feature_dataframe = gpd.GeoDataFrame(df, geometry=gpd.points_from_xy(df.longitude, df.latitude))
        print(self.feature_dataframe)

    def query(self, polygon_points):
        """Points are designed in [(long,lat),...]"""
        polygon = gpd.GeoDataFrame({'geometry': [Polygon(polygon_points)]})
        contained_features = gpd.sjoin(self.feature_dataframe, polygon, op='within')
        response = []
        for index, feature in contained_features.iterrows():
            response.append((feature["name"], feature["longitude"], feature["latitude"], feature["fid"]))
        return response


if __name__ == '__main__':
    q = Query()
    test_features = q.query([(-70.855, 43.123), (-70.855, 43.12), (-70.863, 43.12), (-70.863, 43.123)])
    print(test_features)

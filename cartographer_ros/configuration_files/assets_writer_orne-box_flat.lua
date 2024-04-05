-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.


----------------------------------------------------------
--三次元測域センサからのデータで二次元地図を作成する設定--
----------------------------------------------------------


VOXEL_SIZE = 0.05

include "transform.lua"

options = {

  --pbstream作成時に指定したtracking_frameと同じものを指定
  tracking_frame = "imu_link",


  pipeline = {
    {
      --地図書き出し時に使用する測距値の範囲を指定
      action = "min_max_range_filter",
      min_range = 1.,
      max_range = 50.,
    },
    {
      --地図書き出し時に使用するz軸方向の範囲を指定
      action = "vertical_range_filter",
      min_z = -0.05,
      max_z = 0.2
    },
    {
      --移動している物体の点群を削除 
      action = "voxel_filter_and_remove_moving_objects",
      voxel_size = 0.2,
    },
    {
      action = "dump_num_points",
    },

    {
      --rosで使用される形式(pgmの占有格子地図)で出力
      action = "write_ros_map",
      filestem = "map",
      resolution = 0.2,
      range_data_inserter = {
        insert_free_space = true,
        hit_probability = 0.55,
        miss_probability = 0.49,
      },
    },
  }
}

return options

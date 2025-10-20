###
 # @Author: GetOverMassif 164567487@qq.com
 # @Date: 2022-10-11 13:50:58
 # @LastEditors: GetOverMassif 164567487@qq.com
 # @LastEditTime: 2022-10-13 19:14:25
 # @FilePath: /DSP-SLAM/scripts/run_redwood.sh
 # @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
### 


./dsp_slam_rgbd  \
Vocabulary/ORBvoc.bin  \
configs/ICL_NUIM_home_2.yaml  \
/home/robotlab/dataset/ICL-NUIM/living_room_traj2n_frei_png \
/home/robotlab/dataset/ICL-NUIM/living_room_traj2n_frei_png/associations.txt \
map/self/GroundObjects
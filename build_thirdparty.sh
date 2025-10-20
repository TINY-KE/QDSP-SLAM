
function git_clone(){
  repo_dir=`basename "$1" .git`
  git -C "$repo_dir" pull 2> /dev/null || eval "$1"
}

source Thirdparty/bashcolors/bash_colors.sh
function highlight(){
  clr_magentab clr_bold clr_white "$1"
}

highlight "Starting EDSP-SLAM build script ..."
echo "Available parameters:
        --create-g2o-dbow2
        --create-conda-env
        --build-mmdetection3d"


if [[ $* == *--create-g2o-dbow2* ]] ; then
  highlight "Installing g2o ..."
  cd Thirdparty
  cd g2o
  if [ ! -d build ]; then
    mkdir build
  fi
  cd build
  cmake ..
  make -j12
  cd ../..

  highlight "Installing DBoW2 ..."
  cd DBoW2
  if [ ! -d build ]; then
    mkdir build
  fi
  cd build
  cmake -DOpenCV_DIR="/home/robotlab/thirdparty/for_dspslam/opencv/build" ..
  make -j12
  cd ../../..
fi # ----create-g2o-dbow2

if [[ $* == *--create-conda-env* ]] ; then
  highlight "Creating Python environment ..."
  conda env create -f environment_cuda113.yml
fi # --create-conda-env

source ~/conda.sh 
conda_base=$(conda info --base)
source "$conda_base/etc/profile.d/conda.sh"
conda activate dsp-slam

if [[ $* == *--build-mmdetection3d* ]] ; then
  highlight "Installing mmdetection and mmdetection3d ..."
  pip install pycocotools==2.0.1
  pip install mmcv-full==1.4.0 -f https://download.openmmlab.com/mmcv/dist/cu113/torch1.10.0/index.html
  pip install mmdet==2.14.0
  pip install mmsegmentation==0.14.1
  cd Thirdparty
  git_clone "git clone https://github.com/JingwenWang95/mmdetection3d.git"
  cd mmdetection3d
  pip install -v -e .
  cd ../..
fi # --build-mmdetection3d



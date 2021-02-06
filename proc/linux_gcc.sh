rm -rf bin
mkdir bin
cd bin

proj_name=path_tracing
proj_root_dir=$(pwd)/../

flags=(
  -std=c++17 -Wall -lglfw -lGL -lX11 -pthread -lXi
)

inc=(
  -I ../source/
)

src=(
  ../source/main.cpp
)

g++ -g ${inc[*]} ${src[*]} ${flags[*]}  -lm -o ${proj_name}

cd ..

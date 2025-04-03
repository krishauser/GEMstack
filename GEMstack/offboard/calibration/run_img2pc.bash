root='/mnt/GEMstack'

python3 $root/GEMstack/offboard/calibration/img2pc.py \
    --img_path $root/data/calib1/img/fl/fl11.png \
    --pc_path $root/data/calib1/pc/ouster11.npz \
    --pc_transform_path $root/GEMstack/knowledge/calibration/gem_e4_ouster.yaml \
    --img_intrinsics_path $root/GEMstack/knowledge/calibration/gem_e4_oak_in.yaml \
    --n_features 4 \
    --out_path $root/GEMstack/knowledge/calibration/gem_e4_oak.yaml 
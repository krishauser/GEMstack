data=$1

wget -c -O sam_vit_h.pth https://dl.fbaipublicfiles.com/segment_anything/sam_vit_h_4b8939.pth 

echo 'running segmentation...'
python3 segment-anything scripts/amg.py --checkpoint sam_vit_h.pth --model-type vit_h --input $data/images --output $data/masks/ --stability-score-thresh 0.9 --box-nms-thresh 0.5 --stability-score-offset 0.9

echo 'reprocess segmentation...'
python3 CalibAnything/processed_mask.py -i $data/masks -o $data/processed_masks/

echo 'running calibration...'
CalibAnything/bin/run_lidar2camera $data/calib.json
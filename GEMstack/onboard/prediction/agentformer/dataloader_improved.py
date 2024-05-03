from data.nuscenes_pred_split import get_nuscenes_pred_split
import os, random, numpy as np, copy

from preprocessor_improved import preprocess
from utils.utils import print_log


class data_generator(object):
    def __init__(self, parser, gt_data, log, split="train", phase="training"):
        self.past_frames = parser.past_frames
        self.min_past_frames = parser.min_past_frames
        self.frame_skip = parser.get("frame_skip", 1)
        self.phase = phase
        self.split = split
        assert phase in ["training", "testing"], "error"
        assert split in ["train", "val", "test"], "error"

        process_func = preprocess

        self.num_total_samples = 0
        self.num_sample_list = []
        self.sequence = []

        preprocessor = process_func(parser, gt_data, log, self.split, self.phase)

        num_seq_samples = (
            preprocessor.num_fr
            - (parser.min_past_frames + parser.min_future_frames - 1) * self.frame_skip
        )
        self.num_total_samples += num_seq_samples
        self.num_sample_list.append(num_seq_samples)
        self.sequence.append(preprocessor)

        self.sample_list = list(range(self.num_total_samples))
        self.index = 0
        print_log(f"total num samples: {self.num_total_samples}", log)
        print_log(
            "------------------------------ done --------------------------------\n",
            log=log,
        )

    def shuffle(self):
        random.shuffle(self.sample_list)

    def get_seq_and_frame(self, index):
        index_tmp = copy.copy(index)
        for seq_index in range(len(self.num_sample_list)):  # 0-indexed
            if index_tmp < self.num_sample_list[seq_index]:
                frame_index = (
                    index_tmp
                    + (self.min_past_frames - 1) * self.frame_skip
                    + self.sequence[seq_index].init_frame
                )  # from 0-indexed list index to 1-indexed frame index (for mot)
                return seq_index, frame_index
            else:
                index_tmp -= self.num_sample_list[seq_index]

        assert False, "index is %d, out of range" % (index)

    def is_epoch_end(self):
        if self.index >= self.num_total_samples:
            self.index = 0  # reset
            return True
        else:
            return False

    def next_sample(self):
        sample_index = self.sample_list[self.index]
        seq_index, frame = self.get_seq_and_frame(sample_index)
        seq = self.sequence[seq_index]
        self.index += 1

        data = seq(frame)
        return data

    def __call__(self):
        return self.next_sample()

import glob
import shutil
import os
import random

def move_files_to_directory(dst_folder, files):
    for file in files:
        dst_path = dst_folder + os.path.basename(file)
        if file != dst_path:
            shutil.move(file, dst_path)
def data_split(paths, dst_folder, addition_number_of_records = 0):
    random.shuffle(paths)
    numbers_of_records = len(paths) + addition_number_of_records

    # data split 75% - traing, 15% - validation, 10% - test
    training_split = int(0.75 * numbers_of_records)
    validation_split = training_split + int(0.15 * numbers_of_records)

    training_data = paths[: training_split]
    validation_data = paths[training_split: validation_split]
    test_data = paths[validation_split:]

    move_files_to_directory(dst_folder["training"], training_data)
    move_files_to_directory(dst_folder["validation"], validation_data)
    move_files_to_directory(dst_folder["test"], test_data)

if __name__ == '__main__':
    paths = glob.glob("data/train/*.tfrecord") + glob.glob("data/val/*.tfrecord")
    addition_number_of_records = 3 # since test folder is already contains 3 records
    dst_folder = {"training": "data/train/", "validation": "data/val/", "test": "data/test/"}
    data_split(paths, dst_folder, addition_number_of_records)
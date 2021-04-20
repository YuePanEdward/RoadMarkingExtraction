single_file=dummy_folder/dummy_file.las

road_type=highway
#road_type=urban

#gdb --args 
./bin/roadmarking ${single_file} ${single_file}_out ./model_pool/models_${road_type}_example/ ./config/parameter_${road_type}_example.txt
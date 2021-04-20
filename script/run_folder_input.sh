#gdb --args 

#folder path 
infolder=dummy_folder_in/
outfolder=dummy_folder_out/

#road_type=highway
road_type=urban

for file in ${infolder}/*.las
do 
	./bin/roadmarking ${file} ${outfolder} ./model_pool/models_${road_type}_example/ ./config/parameter_${road_type}_example.txt
done


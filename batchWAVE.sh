#! /bin/sh

./configure

make MODE=release

cd examples/veins

# seed = date +%s
# for i in 05 10 15 20 25 30 35 40 45 50 55 60
for i in 05 10 15 20 25 30 35 40
do
    for j in 0 1 2 3 4 5 6 7 8 9
    # for j in 250
    do
		# ./run -u Cmdenv -f veins_TMAC.ini --seed-set=$i -c NumVeh$j
		./run -u Cmdenv -f omnetpp_express_WAVE.ini -r $j -c Density_$i
	
		mv result/WAVE_App_Result.csv result/WAVE_App_Result_d_"$i"_s_"$j".csv
		mv result/WAVE_Mac_Result.csv result/WAVE_Mac_Result_d_"$i"_s_"$j".csv
    done
done

date


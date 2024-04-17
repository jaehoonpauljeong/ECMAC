#! /bin/sh

make clean

./configure

make -j 4 MODE=release

cd examples/veins

# seed = date +%s
for i in 05 10 15 20 25 30 35 40 45 50 55 60 #65 70 75 80
# for i in 45 50 55 60
do
    for j in 0 #1 2 3 4 5 6 7 8 9
    # for j in 250
    do
		# ./run -u Cmdenv -f veins_TMAC.ini --seed-set=$i -c NumVeh$j
		./run -u Cmdenv -f omnetpp_express_SCMAC.ini -r $j -c Density_$i
	
		mv result/SCMAC_App_Result.csv result/SCMAC_App_Result_SentUnicastPkt_TSAlgo_d_"$i"_s_"$j".csv
		# mv result/SCMAC_Mac_Result.csv result/SCMAC_Mac_Result_d_"$i"_s_"$j".csv
    done
done

date


#!/bin/sh
#touch out
rm -fr ./out
mkdir -p ./out

for File in `find . -name 'station1*'`; 
do 
	outFile=${File//\//\_}
	outFile=${outFile/./}
	outFile='./out'${outFile/\_/\/}'.out'

	echo 'Converting '$File' to '$outFile' ...'
	python adsbC21v5.py txt $File > $outFile 
done
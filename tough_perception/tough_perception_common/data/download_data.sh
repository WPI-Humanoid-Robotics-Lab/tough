#!/bin/bash
fileid="1LCT5EHCRn4sjoGKZaqsdqZc9TxJSfpCT"
filename="FARCENTER.bag"
curl -c ./cookie -s -L "https://drive.google.com/uc?export=download&id=${fileid}" > /dev/null
curl -Lb ./cookie "https://drive.google.com/uc?export=download&confirm=`awk '/download/ {print $NF}' ./cookie`&id=${fileid}" -o ${filename}

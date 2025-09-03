# !/bin/bash
while true; do
    rsync -azhve ssh edhitha@192.168.144.50:/home/edhitha/DCIM/test_cam/images /Users/aahil/Edhitha/edhithaGCS-main-UI-server/Data/Test/
    sleep 1
done

# Do not run this as one script, each block should be executed in separate console! 

## create FIFO
bash recreate_fifo.sh

## set ip
bash set_ip.sh

## ping if connection is good
bash ping.sh

## open communication
`
cd z1_controller/build
./z1_ctrl  
`

## move z1 arm under franka
`
cd z1_sdk/build
./lowcmd_development
`

## provoke both robots

`
./go.sh
`

docker rm -f anglersking_auto_car
docker build -t anglersking_auto_car .

docker run -dit --net=host --name anglersking_auto_car anglersking_auto_car
# --no-cache 
#  --ulimit nofile=262144:262144 -p 9000:9000 

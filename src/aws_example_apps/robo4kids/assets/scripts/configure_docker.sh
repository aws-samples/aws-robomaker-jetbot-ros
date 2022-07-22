# The following command logs in to the an AWS Elastic Container Repository (ECR) to
# enable your machine to pull a base docker image

$(aws ecr get-login --no-include-email --registry-ids 593875212637 --region us-east-1)

# Install Ubuntu dependencies for cross compilation:
apt-get update
apt-get install -y qemu-user-static
if [ $? -ne 0 ]; then
    echo "Failed to install QEMU. Please wait for a few minutese and re-run this script."
    exit 1
fi

#Build Docker Container
echo "Building docker image for robot ..."
docker build -t jetbot-ros -f Dockerfile .

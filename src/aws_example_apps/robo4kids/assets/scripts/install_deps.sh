#!/bin/bash
# if [ $# -ne 1 ]
#     then
#         echo "Please add cloudformation template name as an argument when running install_deps.sh"
#         echo "You can find this in the CloudFormation Console, it will start with mod prefix"
#         echo "For example: install_deps.sh mod-47118164636e49dc"
#         echo
#         exit 1
# fi

WORK_DIR=$(pwd)

# ROBOT_CERTS_FOLDER=$WORK_DIR/../../robot_ws/src/jetbot_app/config
# [ ! -d "$ROBOT_CERTS_FOLDER" ] && mkdir -p $ROBOT_CERTS_FOLDER
SIM_CERTS_FOLDER=$WORK_DIR/../../config
[ ! -d "$SIM_CERTS_FOLDER" ] && mkdir -p $SIM_CERTS_FOLDER
IOTPOLICY="file://../policies/iotpolicy.json"
IOTPOLICYNAME="JetBotPolicy"

# PROJECTNAME=$1
ROBOMAKERFILE="../../roboMakerSettings.json"
AWSCREDSFILE="../../robo4kids_webapp/src/aws-exports.js"
AWSIOTSFILE="../../robo4kids_webapp/src/aws-iot.js"
TELEOP2FILE="../../nodes/teleop.py"





#Add cloudformation outputs as variables to use in the rest of this script
# CF=$(\
# aws cloudformation describe-stacks \
# --stack-name $PROJECTNAME \
# --query 'Stacks[].Outputs[?OutputKey==`SubmitJobSH`].[OutputValue]' \
# --output text
# )
# if [ $? -eq 0 ]
# then
#         echo "Cloud formation stack $PROJECTNAME found"
# else
#         echo "Cloud formation stack $PROJECTNAME not found - exiting"
#         exit
# fi
# for key in $CF
# do
#         echo $key >> addrobomakerresources.sh
# done
#
# source addrobomakerresources.sh
# rm addrobomakerresources.sh

#Get IoT Endpoint to update the robomakersettings.json and awscreds.js files


echo "Get Iot Endpoint ..."

IOTENDPOINT=$(\
aws iot describe-endpoint \
--endpoint-type iot:Data-ATS \
--query 'endpointAddress' \
--output text
)

# #Update roboMakerSettings file
# echo "Updating roboMakerSettings.json ..."
# sed -i "s/<Update S3 Bucketname Here>/$BUCKET_NAME/g" $ROBOMAKERFILE
# sed -i "s|<Update IAM Role ARN Here>|$ARN_SIM_ROLE|g" $ROBOMAKERFILE
# sed -i "s/<Update IoT Endpoint Here>/$IOTENDPOINT/g" $ROBOMAKERFILE
# sed -i "s/<Update Public Subnet 1 Here>/$SUBNET1/g" $ROBOMAKERFILE
# sed -i "s/<Update Public Subnet 2 Here>/$SUBNET2/g" $ROBOMAKERFILE
# sed -i "s/<Update Security Group Here>/$SECURITY_GROUP/g" $ROBOMAKERFILE
# cp $ROBOMAKERFILE /home/ubuntu/environment

#Create Identity aws_cognito_identity_pool_id

echo "Create IdentityPool.."


IDENTITYPOOLID=$(\
aws cognito-identity create-identity-pool \
--identity-pool-name "robo4kidsIdentityPool" \
--allow-unauthenticated-identities \
--query "[IdentityPoolId]" \
--output text
)


echo "Identity Pool ID $IDENTITYPOOLID"



#Get Idendity pool unauth role

# echo "Get UnAuth Role of IdentityPool .."


# UNAUTHROLE=$(\
# aws cognito-identity get-identity-pool-roles \
# --identity-pool-id "$IDENTITYPOOLID" \
# --query "Roles.unauthenticated" \
# --output text
# )

# echo "UnauthRol arn $UNAUTHROLE"

UNAUTHROLENAME="unauthrol-robo4kids"


UNAUTHROLEARN=$(\
aws iam create-role \
--role-name "$UNAUTHROLENAME" \
--assume-role-policy-document \
'{
  "Version": "2012-10-17",
  "Statement": [
    {
      "Effect": "Allow",
      "Principal": {
        "Federated": "cognito-identity.amazonaws.com"
      },
      "Action": "sts:AssumeRoleWithWebIdentity",
      "Condition": {
        "ForAnyValue:StringLike": {
          "cognito-identity.amazonaws.com:amr": "unauthenticated"
        }
      }
    }
  ]
}' \
--query "Role.Arn" \
--output text
)


# add policy to role\


echo  "atach Policy to IAM role .."


aws iam attach-role-policy \
--role-name "$UNAUTHROLENAME" \
--policy-arn "arn:aws:iam::aws:policy/AWSIoTConfigAccess"

aws iam attach-role-policy \
--role-name "$UNAUTHROLENAME" \
--policy-arn "arn:aws:iam::aws:policy/AWSIoTDataAccess"


aws cognito-identity set-identity-pool-roles \
  --identity-pool-id "$IDENTITYPOOLID" \
  --roles unauthenticated="$UNAUTHROLEARN"


SIM_CERTARN=$(\
aws iot create-keys-and-certificate --set-as-active \
--certificate-pem-outfile "$SIM_CERTS_FOLDER/certificate.pem.crt" \
--private-key-outfile  "$SIM_CERTS_FOLDER/private.pem.key" \
--public-key-outfile  "$SIM_CERTS_FOLDER/public.pem.key" \
--query "[certificateArn]" \
--output text
)


echo "Iot Endpoint $IOTENDPOINT"
#Update aws-exports.js
echo "Updating aws-exports.js ..."
AWSREGION=$(aws configure get region)
sed -i "s/<Update PoolId Here>/$IDENTITYPOOLID/g" $AWSCREDSFILE
sed -i "s/<Update Region Here>/$AWSREGION/g" $AWSCREDSFILE


#Update aws-iot.js
echo "Iot Endpoint $IOTENDPOINT"
echo "Updating aws-iot.js ..."
AWSREGION=$(aws configure get region)
sed -i "s/<Update IoT Endpoint Here>/$IOTENDPOINT/g" $AWSIOTSFILE
sed -i "s/<Update Region Here>/$AWSREGION/g" $AWSIOTSFILE


#Update TELEOP.js
echo "Updating teleop.py ..."
AWSREGION=$(aws configure get region)
sed -i "s/<Update IoT Endpoint Here>/$IOTENDPOINT/g" $TELEOP2FILE




#Create IoT Policy
aws iot create-policy \
--policy-name $IOTPOLICYNAME \
--policy-document $IOTPOLICY




#Create IoT Certificates
#Create two certs for robot_ws and simulation_ws
# echo "Creating certificates for robot_ws workspace ..."
# ROBOT_CERTARN=$(\
# aws iot create-keys-and-certificate --set-as-active \
# --certificate-pem-outfile "$ROBOT_CERTS_FOLDER/certificate.pem.crt" \
# --private-key-outfile  "$ROBOT_CERTS_FOLDER/private.pem.key" \
# --public-key-outfile  "$ROBOT_CERTS_FOLDER/public.pem.key" \
# --query "[certificateArn]" \
# --output text
# )

# wget -O $ROBOT_CERTS_FOLDER/root.ca.pem https://www.amazontrust.com/repository/AmazonRootCA1.pem

echo "Creating certificates for simulation_ws workspace ..."
SIM_CERTARN=$(\
aws iot create-keys-and-certificate --set-as-active \
--certificate-pem-outfile "$SIM_CERTS_FOLDER/certificate.pem.crt" \
--private-key-outfile  "$SIM_CERTS_FOLDER/private.pem.key" \
--public-key-outfile  "$SIM_CERTS_FOLDER/public.pem.key" \
--query "[certificateArn]" \
--output text
)

wget -O $SIM_CERTS_FOLDER/root.ca.pem https://www.amazontrust.com/repository/AmazonRootCA1.pem

chmod 755 $SIM_CERTS_FOLDER/*
# chmod 755 $ROBOT_CERTS_FOLDER/*

#attach policy to the certificates
aws iot attach-policy \
--policy-name $IOTPOLICYNAME \
--target $ROBOT_CERTARN

aws iot attach-policy \
--policy-name $IOTPOLICYNAME \
--target $SIM_CERTARN


#Add ROS dependencies
echo "Updating ROS dependencies ..."
cp -a deps/* /etc/ros/rosdep/sources.list.d/
echo "yaml file:///$WORK_DIR/jetbot.yaml" > /etc/ros/rosdep/sources.list.d/21-customdepenencies.list
sudo -u ubuntu rosdep update


#fix ros permissions
rosdep fix-permissions

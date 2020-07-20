singleton Material(german_road_signs_solid)
{
    mapTo = "german_road_signs_solid";
    doubleSided = "1";
    translucentBlendOp = "None";
    detailScale[0] = "2 2";
    materialTag0 = "beamng";
    useAnisotropic[0] = "1";
	annotation = "BUILDINGS";
   colorMap[0] = "/levels/urban/art/objects/german_road_signs_solid_d.dds";
   pixelSpecular[0] = "1";
   vertColor[0] = "1";
};

singleton Material(trafficlight)
{
    mapTo = "trafficlight";
	specularPower[0] = "32";
    pixelSpecular[0] = "1";
    diffuseColor[0] = "1 1 1 1";
    useAnisotropic[0] = "1";
	castShadows = "0";
    translucent = "0";
	glow[0] = "0";
    translucentBlendOp = "None";
    alphaTest = "0";
    alphaRef = "0";
	animFlags[0] = "0x00000010";
    sequenceFramePerSec[0] = "10";
    sequenceSegmentSize[0] = "0.0014";
    materialTag0 = "beamng"; materialTag1 = "vehicle";
    annotation = "TRAFFIC_SIGNALS";
};

singleton Material(trafficlight_off)
{
    mapTo = "trafficlight_off";
	diffuseMap[0] = "levels/urban/art/objects/traffic_cycle.png";
	specularPower[0] = "32";
    pixelSpecular[0] = "1";
    diffuseColor[0] = "1 1 1 1";
    useAnisotropic[0] = "1";
	castShadows = "0";
    translucent = "0";
	glow[0] = "0";
    translucentBlendOp = "None";
    alphaTest = "0";
    alphaRef = "0";
	animFlags[0] = "0x00000010";
    sequenceFramePerSec[0] = "10";
    sequenceSegmentSize[0] = "0.0014";
    materialTag0 = "beamng"; materialTag1 = "vehicle";
    annotation = "TRAFFIC_SIGNALS";
};

//TEXTURES FOR GOLF


singleton Material(_87Golf_BodyMat)
{
   mapTo = "BodyMat";
   doubleSided = "1";
   diffuseMap[2] = "vehicles/87Golf/Textures/87GolfBody_c.dds";
   specularMap[2] = "vehicles/87Golf/Textures/87GolfBody_s.dds";
   diffuseMap[1] = "vehicles/87Golf/Textures/87GolfBody_d.dds";
   specularMap[1] = "vehicles/87Golf/Textures/87GolfBody_s.dds";
   diffuseMap[0] = "vehicles/common/null.dds";
   specularMap[0] = "vehicles/common/null.dds";
   specularPower[0] = "16";
   pixelSpecular[0] = "1";
   specularPower[1] = "16";
   pixelSpecular[1] = "1";
   diffuseColor[0] = "1 1 1 1";
   diffuseColor[1] = "1 1 1 1";

//Diamond Silver Metallic
   //diffuseColor[2] = "0.541176 0.541176 0.541176 1";
//Blue Mica
   //diffuseColor[2] = "0 0.00784314 0.219608 1";
//Red Mica
   //diffuseColor[2] = "0.403922 0.0313726 0.00392157 1";
//Tornado Red
   //diffuseColor[2] = "0.882353 0.0705882 0.00784314 1";
//World Rally Blue (Custom)
   //diffuseColor[2] = "0 0.225 0.808 1";
//Neon Green (Custom)
   //diffuseColor[2] = "0.051 0.416 0.00392 1";
//Black Graphite Metallic (Custom)
   //diffuseColor[2] = "0.028 0.035 0.035 1";
//Player-Generated Color
   diffuseColor[2] = "0 0.05 0.2 1";

   useAnisotropic[0] = "1";
   useAnisotropic[1] = "1";
   useAnisotropic[2] = "1";
   castShadows = "1";
   translucent = "1";
   translucentBlendOp = "None";
   alphaTest = "1";
   alphaRef = "0";
   cubemap = "BNG_Sky_02_cubemap";
   beamngDiffuseColorSlot = 2;
   materialTag0 = "beamng"; materialTag1 = "vehicle";
};

singleton Material(_87Golf_PartsMat)
{
   mapTo = "PartsMat";
   doubleSided = "1";
   diffuseMap[2] = "vehicles/87Golf/Textures/87GolfParts_c.dds";
   specularMap[2] = "vehicles/87Golf/Textures/87GolfParts_s.dds";
   diffuseMap[1] = "vehicles/87Golf/Textures/87GolfParts_d.dds";
   specularMap[1] = "vehicles/87Golf/Textures/87GolfParts_s.dds";
   diffuseMap[0] = "vehicles/common/null.dds";
   specularMap[0] = "vehicles/common/null.dds";
   specularPower[0] = "16";
   pixelSpecular[0] = "1";
   specularPower[1] = "16";
   pixelSpecular[1] = "1";
   diffuseColor[0] = "1 1 1 1";
   diffuseColor[1] = "1 1 1 1";

//Diamond Silver Metallic
   //diffuseColor[2] = "0.541176 0.541176 0.541176 1";
//Blue Mica
   //diffuseColor[2] = "0 0.00784314 0.219608 1";
//Red Mica
   //diffuseColor[2] = "0.403922 0.0313726 0.00392157 1";
//Tornado Red
   //diffuseColor[2] = "0.882353 0.0705882 0.00784314 1";
//World Rally Blue (Custom)
   //diffuseColor[2] = "0 0.225 0.808 1";
//Neon Green (Custom)
   //diffuseColor[2] = "0.051 0.416 0.00392 1";
//Black Graphite Metallic (Custom)
   //diffuseColor[2] = "0.028 0.035 0.035 1";
//Player-Generated Color
   diffuseColor[2] = "0 0.05 0.2 1";

   useAnisotropic[0] = "1";
   useAnisotropic[1] = "1";
   useAnisotropic[2] = "1";
   castShadows = "1";
   translucent = "1";
   translucentBlendOp = "None";
   alphaTest = "1";
   alphaRef = "0";
   cubemap = "BNG_Sky_02_cubemap";
   beamngDiffuseColorSlot = 2;
   materialTag0 = "beamng"; materialTag1 = "vehicle";
};

singleton Material(_87Golf_InteriorMat)
{
   mapTo = "InteriorMat";
   doubleSided = "1";
   diffuseMap[2] = "vehicles/87Golf/Textures/87GolfInterior_c.dds";
   specularMap[2] = "vehicles/87Golf/Textures/87GolfInterior_s.dds";
   diffuseMap[1] = "vehicles/87Golf/Textures/87GolfInterior_d.dds";
   specularMap[1] = "vehicles/87Golf/Textures/87GolfInterior_s.dds";
   diffuseMap[0] = "vehicles/common/null.dds";
   specularMap[0] = "vehicles/common/null.dds";
   specularPower[0] = "16";
   pixelSpecular[0] = "1";
   specularPower[1] = "16";
   pixelSpecular[1] = "1";
   diffuseColor[0] = "1 1 1 1";
   diffuseColor[1] = "1 1 1 1";
   diffuseColor[2] = "0.5 0.5 0.5 0.1";
   useAnisotropic[0] = "1";
   useAnisotropic[1] = "1";
   useAnisotropic[2] = "1";
   castShadows = "1";
   translucent = "1";
   translucentBlendOp = "None";
   alphaTest = "1";
   alphaRef = "0";
   cubemap = "BNG_Sky_02_cubemap";
   beamngDiffuseColorSlot = 0;
   materialTag0 = "beamng"; materialTag1 = "vehicle";
};

singleton Material(_87Golf_MiscMat)
{
   mapTo = "MiscMat";
   doubleSided = "1";
   diffuseMap[2] = "vehicles/87Golf/Textures/87GolfMisc_c.dds";
   specularMap[2] = "vehicles/87Golf/Textures/87GolfMisc_s.dds";
   diffuseMap[1] = "vehicles/87Golf/Textures/87GolfMisc_d.dds";
   specularMap[1] = "vehicles/87Golf/Textures/87GolfMisc_s.dds";
   diffuseMap[0] = "vehicles/common/null.dds";
   specularMap[0] = "vehicles/common/null.dds";
   specularPower[0] = "16";
   pixelSpecular[0] = "1";
   specularPower[1] = "16";
   pixelSpecular[1] = "1";
   diffuseColor[0] = "1 1 1 1";
   diffuseColor[1] = "1 1 1 1";
   diffuseColor[2] = "0.5 0.5 0.5 0.1";
   useAnisotropic[0] = "1";
   useAnisotropic[1] = "1";
   useAnisotropic[2] = "1";
   castShadows = "1";
   translucent = "1";
   translucentBlendOp = "None";
   alphaTest = "1";
   alphaRef = "0";
   cubemap = "BNG_Sky_02_cubemap";
   beamngDiffuseColorSlot = 0;
   materialTag0 = "beamng"; materialTag1 = "vehicle";
};

singleton Material(_87Golf_Chrome)
{
   mapTo = "Chrome";
   doubleSided = "1";
   diffuseMap[2] = "vehicles/87Golf/87Golf_c.dds";
   specularMap[2] = "vehicles/87Golf/87Golf_s.dds";
   //normalMap[2] = "vehicles/87Golf/87Golf_n.dds";
   diffuseMap[1] = "vehicles/87Golf/87Golf_d.dds";
   specularMap[1] = "vehicles/87Golf/87Golf_s.dds";
   normalMap[1] = "vehicles/87Golf/87Golf_n.dds";
   diffuseMap[0] = "vehicles/common/null.dds";
   specularMap[0] = "vehicles/common/null.dds";
   //normalMap[0] = "vehicles/87Golf/87Golf_n.dds";
   //diffuseMap[3] = "vehicles/87Golf/87Golf_dirt.dds";
   //normalMap[3] = "vehicles/87Golf/87Golf_n.dds";
   specularPower[0] = "48";
   pixelSpecular[0] = "1";
   specularPower[1] = "48";
   pixelSpecular[1] = "1";
   diffuseColor[0] = "1 1 1 1";
   diffuseColor[1] = "1 1 1 1";
   diffuseColor[2] = "0.455 0.455 0.455 1";
   useAnisotropic[0] = "1";
   useAnisotropic[1] = "1";
   useAnisotropic[2] = "1";
   //diffuseColor[3] = "1.5 1.5 1.5 1";
   castShadows = "1";
   translucent = "1";
   translucentBlendOp = "None";
   alphaTest = "1";
   alphaRef = "0";
   cubemap = "BNG_Sky_02_cubemap";
   beamngDiffuseColorSlot = 0;
   materialTag0 = "beamng"; materialTag1 = "vehicle";
};

singleton Material(_87Golf_IntMetal)
{
   mapTo = "IntMetal";
   doubleSided = "1";
   diffuseMap[2] = "vehicles/87Golf/87Golf_c.dds";
   specularMap[2] = "vehicles/87Golf/87Golf_s.dds";
   //normalMap[2] = "vehicles/87Golf/87Golf_n.dds";
   diffuseMap[1] = "vehicles/87Golf/87Golf_d.dds";
   specularMap[1] = "vehicles/87Golf/87Golf_s.dds";
   normalMap[1] = "vehicles/87Golf/87Golf_n.dds";
   diffuseMap[0] = "vehicles/common/null.dds";
   specularMap[0] = "vehicles/common/null.dds";
   //normalMap[0] = "vehicles/87Golf/87Golf_n.dds";
   //diffuseMap[3] = "vehicles/87Golf/87Golf_dirt.dds";
   //normalMap[3] = "vehicles/87Golf/87Golf_n.dds";
   specularPower[0] = "10";
   pixelSpecular[0] = "1";
   specularPower[1] = "10";
   pixelSpecular[1] = "1";
   diffuseColor[0] = "1 1 1 1";
   diffuseColor[1] = "1 1 1 1";
   diffuseColor[2] = "0.69 0.69 0.69 1";
   useAnisotropic[0] = "1";
   useAnisotropic[1] = "1";
   useAnisotropic[2] = "1";
   //diffuseColor[3] = "1.5 1.5 1.5 1";
   castShadows = "1";
   translucent = "1";
   translucentBlendOp = "None";
   alphaTest = "1";
   alphaRef = "0";
   cubemap = "BNG_Sky_02_cubemap";
   beamngDiffuseColorSlot = 0;
   materialTag0 = "beamng"; materialTag1 = "vehicle";
};

singleton Material(_87Golf_FrontSeatMat)
{
   mapTo = "FrontSeatMat";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfFrontSeats.jpg";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};

singleton Material(_87Golf_HeadlightsMat)
{
   mapTo = "HeadlightsMat";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfHeadlights";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};

singleton Material(_87Golf_DashMat)
{
   mapTo = "DashMat";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfDash";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};

singleton Material(_87Golf_SeatFLMat)
{
   mapTo = "SeatFLMat";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfSeatFL";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};

singleton Material(_87Golf_SeatFRMat)
{
   mapTo = "SeatFRMat";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfSeatFL";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};

singleton Material(_87Golf_RearSeatMat)
{
   mapTo = "RearSeatMat";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfRearSeats";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};

singleton Material(_87Golf_SteerMat)
{
   mapTo = "SteerMat";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfSteer";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};

singleton Material(_87Golf_SStalkMat)
{
   mapTo = "SStalkMat";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfBlackPlastics";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};

singleton Material(_87Golf_WStalkMat)
{
   mapTo = "WStalkMat";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfBlackPlastics";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};

singleton Material(_87Golf_DoorLMat)
{
   mapTo = "DoorLMat";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfDoorL";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};

singleton Material(_87Golf_DoorRMat)
{
   mapTo = "DoorRMat";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfDoorR";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};

singleton Material(_87Golf_HoodMat)
{
   mapTo = "HoodMat";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfHood";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};

singleton Material(_87Golf_TrunkMat)
{
   mapTo = "TrunkMat";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfTrunk";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};

singleton Material(_87Golf_FasciaMat)
{
   mapTo = "FasciaMat";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfFascia";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};

singleton Material(_87Golf_MShiftMat)
{
   mapTo = "MShiftMat";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfShift";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};

singleton Material(_87Golf_AShiftMat)
{
   mapTo = "AShiftMat";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfShift";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};

singleton Material(_87Golf_PBMat)
{
   mapTo = "PBMat";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfPB";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};

singleton Material(_87Golf_DPRMat)
{
   mapTo = "DPRMat";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfShift";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};

singleton Material(_87Golf_DPLMat)
{
   mapTo = "DPLMat";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfShift";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};

singleton Material(_87Golf_CPMat)
{
   mapTo = "CPMat";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfShift";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};

singleton Material(_87Golf_BPMat)
{
   mapTo = "BPMat";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfShift";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};

singleton Material(_87Golf_GPMat)
{
   mapTo = "GPMat";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfShift";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};

singleton Material(_87Golf_MirrorIMat)
{
   mapTo = "MirrorIMat";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfMirrors";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};

singleton Material(_87Golf_MirrorLMat)
{
   mapTo = "MirrorLMat";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfMirrors";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};

singleton Material(_87Golf_UAFMat)
{
   mapTo = "UAFMat";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfMetals";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};

singleton Material(_87Golf_LAFMat)
{
   mapTo = "LAFMat";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfMetals";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};

singleton Material(_87Golf_HubFRMat)
{
   mapTo = "HubFRMat";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfMetals";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};

singleton Material(_87Golf_HubFLMat)
{
   mapTo = "HubFLMat";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfMetals";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};

singleton Material(_87Golf_HalfShaftMat)
{
   mapTo = "HalfShaftMat";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfMetals";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};

singleton Material(_87Golf_FuelTankMat)
{
   mapTo = "FuelTankMat";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfFuelTank";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};

singleton Material(_87Golf_ExhaustMat)
{
   mapTo = "ExhaustMat";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfMetals";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};

singleton Material(_87Golf_Engine)
{
   mapTo = "Engine";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfEngine";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};

singleton Material(_87Golf_EngineBayCrap)
{
   mapTo = "EngineBayCrap";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfEngParts";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};

singleton Material(_87Golf_CoilsFMat)
{
   mapTo = "CoilsFMat";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfMetals";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};

singleton Material(_87Golf_GlassMat)
{
   mapTo = "GlassMat";
   diffuseColor[0] = "0.188235 0.188235 0.188235 0.12";
   specularPower[0] = "128";
   doubleSided = "1";
   translucent = "1";
   translucentZWrite = "1";
   alphaTest = "1";
};

singleton Material(_87Golf_LARMat)
{
   mapTo = "LARMat";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfMetals";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};

singleton Material(_87Golf_HubRMat)
{
   mapTo = "HubRMat";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfMetals";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};

singleton Material(_87Golf_CoilsRMat)
{
   mapTo = "CoilsRMat";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfMetals";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};

singleton Material(_87Golf_UARMat)
{
   mapTo = "UARMat";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfMetals";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};

singleton Material(_87Golf_SteerRallyMat)
{
   mapTo = "SteerRallyMat";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfSteerR";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};

singleton Material(_87Golf_HubcapMat)
{
   mapTo = "HubcapMat";
   diffuseMap[0] = "vehicles/87Golf/Textures/87GolfHubcap";
   specular[0] = "0.5 0.5 0.5 1";
   specularPower[0] = "50";
   doubleSided = "1";
   translucentBlendOp = "None";
};




//TEXTURES FOR WHEELS (CREDIT TO DKUTCH AND JALKKU; USED WITH PERMISSION)




singleton Material(Dk_81_720_datsun720_2wdsteelie)
{
   mapTo = "datsun720_2wdsteelie";
   diffuseMap[1] = "vehicles/common/wheels/steelwheel_03a_d.dds";
   specularMap[1] = "vehicles/common/wheels/steelwheel_03a_s.dds";
   normalMap[1] = "vehicles/common/wheels/steelwheel_03a_n.dds";
   diffuseMap[0] = "vehicles/common/null.dds";
   specularMap[0] = "vehicles/common/null.dds";
   normalMap[0] = "vehicles/common/wheels/steelwheel_03a_n.dds";
   specularPower[0] = "16";
   pixelSpecular[0] = "1";
   specularPower[1] = "16";
   pixelSpecular[1] = "1";
   diffuseColor[0] = "1 1 1 1";
   diffuseColor[1] = "1 1 1 1";
   useAnisotropic[0] = "1";
   useAnisotropic[1] = "1";
   castShadows = "1";
   translucent = "1";
   translucentBlendOp = "None";
   alphaTest = "1";
   alphaRef = "0";
   cubemap = "BNG_Sky_02_cubemap";
   materialTag0 = "beamng"; materialTag1 = "vehicle";
};

singleton Material(Dk_81_720_datsun720_2wdtire)
{
   mapTo = "datsun720_2wdtire";
   diffuseMap[1] = "vehicles/common/tires/tire_01a_d.dds";
   specularMap[1] = "vehicles/common/tires/tire_01a_s.dds";
   normalMap[1] = "vehicles/common/tires/tire_01a_n.dds";
   diffuseMap[0] = "vehicles/common/null.dds";
   specularMap[0] = "vehicles/common/null.dds";
   normalMap[0] = "vehicles/common/tires/tire_01a_n.dds";
   //diffuseMap[2] = "vehicles/common/tire_01a_dirt.dds";
   specularPower[0] = "16";
   pixelSpecular[0] = "1";
   specularPower[1] = "16";
   pixelSpecular[1] = "1";
   diffuseColor[0] = "1 1 1 1";
   diffuseColor[1] = "1 1 1 1";
   //diffuseColor[2] = "1.5 1.5 1.5 1";
   useAnisotropic[0] = "1";
   useAnisotropic[1] = "1";
   castShadows = "1";
   translucent = "1";
   translucentBlendOp = "None";
   alphaTest = "1";
   alphaRef = "0";
   cubemap = "BNG_Sky_02_cubemap";
   materialTag0 = "beamng"; materialTag1 = "vehicle";
};

singleton Material(hatchMRi_hatch_MRi_Rwheels)
{
   mapTo = "hatch_MRi_Rwheels";
   diffuseMap[1] = "vehicles/87Golf/Textures/hatchMRi_Rwheels_d.dds";
   //specularMap[1] = "vehicles/87Golf/Textures/ABC_81DatsunKC_tires_aftermarket_s.dds";
   normalMap[1] = "vehicles/87Golf/Textures/hatchMRi_Rwheels_n.dds";
   diffuseMap[0] = "vehicles/common/null.dds";
   specularMap[0] = "vehicles/common/null.dds";
   normalMap[0] = "vehicles/87Golf/Textures/hatchMRi_Rwheels_n.dds";
   //diffuseMap[2] = "vehicles/common/tire_01a_dirt.dds";
   specularPower[0] = "16";
   pixelSpecular[0] = "1";
   specularPower[1] = "16";
   pixelSpecular[1] = "1";
   diffuseColor[0] = "1 1 1 1";
   diffuseColor[1] = "1 1 1 1";
   //diffuseColor[2] = "1.5 1.5 1.5 1";
   useAnisotropic[0] = "1";
   useAnisotropic[1] = "1";
   castShadows = "1";
   translucent = "1";
   translucentBlendOp = "None";
   alphaTest = "1";
   alphaRef = "0";
   cubemap = "BNG_Sky_02_cubemap";
   materialTag0 = "beamng"; materialTag1 = "vehicle";
};

singleton Material(hatchMRi_hatch_MRi_Rtires)
{
   mapTo = "hatch_MRi_Rtires";
   diffuseMap[1] = "vehicles/87Golf/Textures/hatchMRi_Rtires_d.dds";
   specularMap[1] = "vehicles/87Golf/Textures/hatchMRi_Rtires_s.dds";
   normalMap[1] = "vehicles/87Golf/Textures/hatchMRi_Rtires_n.dds";
   diffuseMap[0] = "vehicles/common/null.dds";
   specularMap[0] = "vehicles/common/null.dds";
   normalMap[0] = "vehicles/87Golf/Textures/hatchMRi_Rtires_n.dds";
   //diffuseMap[2] = "vehicles/common/tire_01a_dirt.dds";
   specularPower[0] = "16";
   pixelSpecular[0] = "1";
   specularPower[1] = "16";
   pixelSpecular[1] = "1";
   diffuseColor[0] = "1 1 1 1";
   diffuseColor[1] = "1 1 1 1";
   //diffuseColor[2] = "1.5 1.5 1.5 1";
   useAnisotropic[0] = "1";
   useAnisotropic[1] = "1";
   castShadows = "1";
   translucent = "1";
   translucentBlendOp = "None";
   alphaTest = "1";
   alphaRef = "0";
   cubemap = "BNG_Sky_02_cubemap";
   materialTag0 = "beamng"; materialTag1 = "vehicle";
};

singleton Material(polo_tire_gravel_material)
{
   mapTo = "tire_gravel-material";
   diffuseMap[1] = "vehicles/87Golf/Textures/polo_tire_gravel.png";
   specularMap[1] = "vehicles/87Golf/Textures/polo_tire_s.png";
   normalMap[1] = "vehicles/87Golf/Textures/polo_tire_n.png";
   useAnisotropic[1] = "1";
   //diffuseColor[3] = "1.5 1.5 1.5 1";
   castShadows = "1";
   translucent = "1";
   translucentBlendOp = "None";
   alphaTest = "1";
   alphaRef = "0";
   cubemap = "BNG_Sky_02_cubemap";
   materialTag0 = "beamng"; materialTag1 = "vehicle";
   doublesided = "1";
};

singleton Material(polo_rim_material)
{
   mapTo = "rim-material";
   diffuseMap[1] = "vehicles/87Golf/Textures/polo_rim.png";
   specularMap[1] = "vehicles/87Golf/Textures/polo_rim.png";
   normalMap[1] = "vehicles/87Golf/Textures/polo_rim.png";
   useAnisotropic[1] = "1";
   specularPower[0] = "16";
   pixelSpecular[0] = "1";
   specularPower[1] = "16";
   pixelSpecular[1] = "1";
   diffuseColor[0] = "0.1 0.1 0.1 1";
   diffuseColor[1] = "1 1 1 0.7";
   diffuseColor[2] = "0.06 0.06 0.06 1";
   //diffuseColor[3] = "1.5 1.5 1.5 1";
   castShadows = "1";
   translucent = "1";
   translucentBlendOp = "None";
   alphaTest = "1";
   alphaRef = "0";
   cubemap = "BNG_Sky_02_cubemap";
   materialTag0 = "beamng"; materialTag1 = "vehicle";
   doublesided = "1";
};




//ORIGINAL HATCH MATERIALS (CREDIT TO GABESTER)

singleton Material(hatch)
{
   mapTo = "hatch";
   diffuseMap[2] = "vehicles/hatch/hatch_c.dds";
   specularMap[2] = "vehicles/hatch/hatch_s.dds";
   normalMap[2] = "vehicles/hatch/hatch_n.dds";
   diffuseMap[1] = "vehicles/hatch/hatch_d.dds";
   specularMap[1] = "vehicles/hatch/hatch_s.dds";
   normalMap[1] = "vehicles/hatch/hatch_n.dds";
   diffuseMap[0] = "vehicles/common/null.dds";
   specularMap[0] = "vehicles/common/null.dds";
   normalMap[0] = "vehicles/hatch/hatch_n.dds";
   //diffuseMap[3] = "vehicles/hatch/hatch_dirt.dds";
   //normalMap[3] = "vehicles/hatch/hatch_n.dds";
   specularPower[0] = "16";
   pixelSpecular[0] = "1";
   specularPower[1] = "16";
   pixelSpecular[1] = "1";
   diffuseColor[0] = "1 1 1 1";
   diffuseColor[1] = "1 1 1 1";
   diffuseColor[2] = "0.1 0.4 0.8 1";
   useAnisotropic[0] = "1";
   useAnisotropic[1] = "1";
   useAnisotropic[2] = "1";
   //diffuseColor[3] = "1.5 1.5 1.5 1";
   castShadows = "1";
   translucent = "1";
   translucentBlendOp = "None";
   alphaTest = "1";
   alphaRef = "0";
   cubemap = "BNG_Sky_02_cubemap";
   beamngDiffuseColorSlot = 2;
   materialTag0 = "beamng"; materialTag1 = "vehicle";
};

singleton Material(hatch_interior)
{
   mapTo = "hatch_interior";
   diffuseMap[0] = "hatch_interior_d.dds";
   normalMap[0] = "hatch_interior_n.dds";
   specularPower[0] = "16";
   pixelSpecular[0] = "1";
   useAnisotropic[0] = "1";
   specularMap[0] = "hatch_interior_s.dds";
   materialTag0 = "beamng"; materialTag1 = "vehicle";
   diffuseColor[0] = "0.8 0.8 0.8 1";
};

singleton Material(hatch_gauges)
{
   mapTo = "hatch_gauges";
   diffuseMap[0] = "hatch_gauges_d.dds";
   specularMap[0] = "hatch_gauges_s.dds";
   normalMap[0] = "hatch_gauges_n.dds";
   specularPower[0] = "16";
   pixelSpecular[0] = "1";
   useAnisotropic[0] = "1";
   materialTag0 = "beamng"; materialTag1 = "vehicle";
   diffuseColor[0] = "0.75 0.75 0.75 1";
};

singleton Material(hatch_gauges_on)
{
   mapTo = "hatch_gauges_on";
   diffuseMap[1] = "hatch_gauges_g.dds";
   specularMap[1] = "hatch_gauges_s.dds";
   normalMap[1] = "hatch_gauges_n.dds";
   diffuseMap[0] = "hatch_gauges_d.dds";
   specularMap[0] = "hatch_gauges_s.dds";
   normalMap[0] = "hatch_gauges_n.dds";
   specularPower[0] = "16";
   pixelSpecular[0] = "1";
   glow[1] = "1";
   emissive[1] = "1";
   useAnisotropic[0] = "1";
   useAnisotropic[1] = "1";
   materialTag0 = "beamng"; materialTag1 = "vehicle";
   diffuseColor[0] = "0.75 0.75 0.75 1";
   diffuseColor[1] = "1.5 1.5 1.5 0.2";
};

singleton Material(hatch_glass)
{
   mapTo = "hatch_glass";
   diffuseMap[0] = "vehicles/87Golf/hatch_glass_d.dds";
   specularMap[0] = "vehicles/87Golf/hatch_glass_s.dds";
   diffuseMap[1] = "vehicles/87Golf/hatch_glass_da.dds";
   specularMap[1] = "vehicles/87Golf/hatch_glass_s.dds";
   //diffuseMap[2] = "vehicles/87Golf/hatch_glass_dirt.dds";
   specularPower[0] = "128";
   pixelSpecular[0] = "1";
   specularPower[1] = "128";
   pixelSpecular[1] = "1";
   diffuseColor[0] = "1 1 1 1";
   diffuseColor[1] = "1 1 1 1";
   useAnisotropic[0] = "1";
   useAnisotropic[1] = "1";
   //diffuseColor[2] = "1.5 1.5 1.5 1";
   castShadows = "0";
   translucent = "1";
   alphaTest = "1";
   alphaRef = "0";
   cubemap = "BNG_Sky_02_cubemap";
   materialTag0 = "beamng"; materialTag1 = "vehicle";
};

singleton Material(hatch_lights_on)
{
   mapTo = "hatch_lights_on";
   diffuseMap[2] = "vehicles/87Golf/hatch_lights_g.dds";
   specularMap[2] = "vehicles/87Golf/hatch_lights_s.dds";
   normalMap[2] = "vehicles/87Golf/hatch_lights_n.dds";
   diffuseMap[1] = "vehicles/87Golf/hatch_lights_d.dds";
   specularMap[1] = "vehicles/87Golf/hatch_lights_s.dds";
   normalMap[1] = "vehicles/87Golf/hatch_lights_n.dds";
   diffuseMap[0] = "vehicles/common/null.dds";
   specularMap[0] = "vehicles/common/null.dds";
   normalMap[0] = "vehicles/87Golf/hatch_lights_n.dds";
   //diffuseMap[3] = "vehicles/87Golf/hatch_lights_dirt.dds";
   //normalMap[3] = "vehicles/87Golf/hatch_lights_n.dds";
   specularPower[0] = "16";
   pixelSpecular[0] = "1";
   specularPower[1] = "16";
   pixelSpecular[1] = "1";
   diffuseColor[0] = "1 1 1 1";
   diffuseColor[1] = "1.5 1.5 1.5 1";
   diffuseColor[2] = "1.5 1.5 1.5 0.12";
   useAnisotropic[0] = "1";
   useAnisotropic[1] = "1";
   useAnisotropic[2] = "1";
   //diffuseColor[3] = "1.5 1.5 1.5 1";
   castShadows = "1";
   translucent = "1";
   translucentBlendOp = "None";
   alphaTest = "1";
   alphaRef = "0";
   cubemap = "BNG_Sky_02_cubemap";
   glow[2] = "1";
   emissive[2] = "1";
   materialTag0 = "beamng"; materialTag1 = "vehicle";
};

singleton Material(hatch_lights_on_intense)
{
   mapTo = "hatch_lights_on_intense";
   diffuseMap[2] = "vehicles/87Golf/hatch_lights_g.dds";
   specularMap[2] = "vehicles/87Golf/hatch_lights_s.dds";
   normalMap[2] = "vehicles/87Golf/hatch_lights_n.dds";
   diffuseMap[1] = "vehicles/87Golf/hatch_lights_d.dds";
   specularMap[1] = "vehicles/87Golf/hatch_lights_s.dds";
   normalMap[1] = "vehicles/87Golf/hatch_lights_n.dds";
   diffuseMap[0] = "vehicles/common/null.dds";
   specularMap[0] = "vehicles/common/null.dds";
   normalMap[0] = "vehicles/87Golf/hatch_lights_n.dds";
   //diffuseMap[3] = "vehicles/87Golf/hatch_lights_dirt.dds";
   //normalMap[3] = "vehicles/87Golf/hatch_lights_n.dds";
   specularPower[0] = "16";
   pixelSpecular[0] = "1";
   specularPower[1] = "16";
   pixelSpecular[1] = "1";
   diffuseColor[0] = "1 1 1 1";
   diffuseColor[1] = "1.5 1.5 1.5 1";
   diffuseColor[2] = "1.5 1.5 1.5 0.20";
   useAnisotropic[0] = "1";
   useAnisotropic[1] = "1";
   useAnisotropic[2] = "1";
   //diffuseColor[3] = "1.5 1.5 1.5 1";
   castShadows = "1";
   translucent = "1";
   translucentBlendOp = "None";
   alphaTest = "1";
   alphaRef = "0";
   cubemap = "BNG_Sky_02_cubemap";
   glow[2] = "1";
   emissive[2] = "1";
   materialTag0 = "beamng"; materialTag1 = "vehicle";
};

singleton Material(hatch_glass_dmg)
{
   mapTo = "hatch_glass_dmg";
   diffuseMap[0] = "vehicles/87Golf/hatch_glass_dmg_d.dds";
   specularMap[0] = "vehicles/common/glass_dmg_s.dds";
   normalMap[0] = "vehicles/common/glass_dmg_n.dds";
   //diffuseMap[2] = "vehicles/87Golf/hatch_glass_dirt.dds";
   specularPower[0] = "32";
   diffuseColor[0] = "1.5 1.5 1.5 1";
   useAnisotropic[0] = "1";
   //diffuseColor[2] = "1.5 1.5 1.5 1";
   castShadows = "0";
   translucent = "1";
   alphaTest = "1";
   alphaRef = "0";
   //cubemap = "BNG_Sky_02_cubemap";
   materialTag0 = "beamng"; materialTag1 = "vehicle";
};

singleton Material(hatch_lights)
{
   mapTo = "hatch_lights";
   diffuseMap[1] = "vehicles/87Golf/hatch_lights_d.dds";
   specularMap[1] = "vehicles/87Golf/hatch_lights_s.dds";
   normalMap[1] = "vehicles/87Golf/hatch_lights_n.dds";
   diffuseMap[0] = "vehicles/common/null.dds";
   specularMap[0] = "vehicles/common/null.dds";
   normalMap[0] = "vehicles/87Golf/hatch_lights_n.dds";
   //diffuseMap[2] = "vehicles/87Golf/hatch_lights_dirt.dds";
   //normalMap[2] = "vehicles/87Golf/hatch_lights_n.dds";
   specularPower[0] = "16";
   pixelSpecular[0] = "1";
   specularPower[1] = "16";
   pixelSpecular[1] = "1";
   diffuseColor[0] = "1 1 1 1";
   diffuseColor[1] = "1 1 1 1";
   useAnisotropic[0] = "1";
   useAnisotropic[1] = "1";
   //diffuseColor[2] = "1.5 1.5 1.5 1";
   castShadows = "1";
   translucent = "1";
   translucentBlendOp = "None";
   alphaTest = "1";
   alphaRef = "0";
   diffuseColor[1] = "1.5 1.5 1.5 1";
   cubemap = "BNG_Sky_02_cubemap";
   materialTag0 = "beamng"; materialTag1 = "vehicle";
};

singleton Material(hatch_lights_dmg)
{
   mapTo = "hatch_lights_dmg";
   diffuseMap[1] = "vehicles/87Golf/hatch_lights_dmg_d.dds";
   specularMap[1] = "vehicles/87Golf/hatch_lights_dmg_s.dds";
   normalMap[1] = "vehicles/87Golf/hatch_lights_dmg_n.dds";
   diffuseMap[0] = "vehicles/common/null.dds";
   specularMap[0] = "vehicles/common/null.dds";
   normalMap[0] = "vehicles/87Golf/hatch_lights_dmg_n.dds";
   //diffuseMap[2] = "vehicles/87Golf/hatch_lights_dirt.dds";
   //normalMap[2] = "vehicles/87Golf/hatch_lights_dmg_n.dds";
   specularPower[0] = "16";
   pixelSpecular[0] = "1";
   specularPower[1] = "16";
   pixelSpecular[1] = "1";
   diffuseColor[0] = "1 1 1 1";
   diffuseColor[1] = "1 1 1 1";
   useAnisotropic[0] = "1";
   useAnisotropic[1] = "1";
   //diffuseColor[2] = "1.5 1.5 1.5 1";
   castShadows = "1";
   translucent = "1";
   translucentBlendOp = "None";
   alphaTest = "1";
   alphaRef = "0";
   diffuseColor[1] = "1.5 1.5 1.5 1";
   cubemap = "BNG_Sky_02_cubemap";
   materialTag0 = "beamng"; materialTag1 = "vehicle";
};

singleton Material(hatch_reverselight)
{
   mapTo = "hatch_reverselight";
};

singleton Material(hatch_signal_R)
{
   mapTo = "hatch_signal_R";
};

singleton Material(hatch_signal_L)
{
   mapTo = "hatch_signal_L";
};

singleton Material(hatch_headlight)
{
   mapTo = "hatch_headlight";
};

singleton Material(hatch_parkinglight)
{
   mapTo = "hatch_parkinglight";
};

singleton Material(hatch_chmsl)
{
   mapTo = "hatch_chmsl";
};

singleton Material(hatch_taillight)
{
   mapTo = "hatch_taillight";
};

singleton Material(hatch_highbeam)
{
   mapTo = "highbeam";
};


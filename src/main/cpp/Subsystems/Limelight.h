#ifndef LIMELIGHTHELPERS_H
#define LIMELIGHTHELPERS_H

///
//https://github.com/LimelightVision/limelightlib-wpicpp
///

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableValue.h"
#include "wpi/json.h"

#include <chrono>
#include <iostream>
#include <string>
#include <unistd.h>
#include <vector>

namespace LimelightHelpers
{
    inline std::string SanitizeName(const std::string &name)
    {
        if (name == "")
        {
            return "limelight";
        }
        return name;
    }

    inline std::shared_ptr<nt::NetworkTable> GetLimelightNTTable(const std::string &tableName)
    {
        return nt::NetworkTableInstance::GetDefault().GetTable((tableName));
    }

    inline nt::NetworkTableEntry GetLimelightNTTableEntry(const std::string &tableName, const std::string &entryName)
    {
        return GetLimelightNTTable(tableName)->GetEntry(entryName);
    }

    inline double GetLimelightNTDouble(const std::string &tableName, const std::string &entryName)
    {
        return GetLimelightNTTableEntry(tableName, entryName).GetDouble(0.0);
    }

    inline std::vector<double> GetLimelightNTDoubleArray(const std::string &tableName, const std::string &entryName)
    {
        return GetLimelightNTTableEntry(tableName, entryName).GetDoubleArray(std::span<double>{});
    }

    inline std::string GetLimelightNTString(const std::string &tableName, const std::string &entryName)
    {
        return GetLimelightNTTableEntry(tableName, entryName).GetString("");
    }

    inline void SetLimelightNTDouble(const std::string &tableName, const std::string entryName, double val)
    {
        GetLimelightNTTableEntry(tableName, entryName).SetDouble(val);
    }

    inline void SetLimelightNTDoubleArray(const std::string &tableName,
                                          const std::string &entryName,
                                          const std::span<const double> &vals)
    {
        GetLimelightNTTableEntry(tableName, entryName).SetDoubleArray(vals);
    }

    inline double GetTX(const std::string &limelightName = "")
    {
        return GetLimelightNTDouble(limelightName, "tx");
    }

    inline double GetTY(const std::string &limelightName = "")
    {
        return GetLimelightNTDouble(limelightName, "ty");
    }

    inline double GetTA(const std::string &limelightName = "")
    {
        return GetLimelightNTDouble(limelightName, "ta");
    }

    inline double GetLatency_Pipeline(const std::string &limelightName = "")
    {
        return GetLimelightNTDouble(limelightName, "tl");
    }

    inline double GetLatency_Capture(const std::string &limelightName = "")
    {
        return GetLimelightNTDouble(limelightName, "tl_cap");
    }

    inline std::string GetJSONDump(const std::string &limelightName = "")
    {
        return GetLimelightNTString(limelightName, "json");
    }

    inline std::vector<double> GetBotpose(const std::string &limelightName = "")
    {
        return GetLimelightNTDoubleArray(limelightName, "botpose");
    }

    inline std::vector<double> GetBotpose_wpiRed(const std::string &limelightName = "")
    {
        return GetLimelightNTDoubleArray(limelightName, "botpose_wpired");
    }

    inline std::vector<double> GetBotpose_wpiBlue(const std::string &limelightName = "")
    {
        return GetLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
    }

    inline std::vector<double> GetBotpose_TargetSpace(const std::string &limelightName = "")
    {
        return GetLimelightNTDoubleArray(limelightName, "botpose_targetSpace");
    }

    inline std::vector<double> GetCameraPose_TargetSpace(const std::string &limelightName = "")
    {
        return GetLimelightNTDoubleArray(limelightName, "camerapose_targetspace");
    }

    inline std::vector<double> GetTargetPose_CameraSpace(const std::string &limelightName = "")
    {
        return GetLimelightNTDoubleArray(limelightName, "targetpose_cameraspace");
    }

    inline std::vector<double> GetTargetPose_RobotSpace(const std::string &limelightName = "")
    {
        return GetLimelightNTDoubleArray(limelightName, "targetpose_robotspace");
    }

    inline std::vector<double> GetTargetColor(const std::string &limelightName = "")
    {
        return GetLimelightNTDoubleArray(limelightName, "tc");
    }

    inline double GetFiducialID(const std::string &limelightName = "")
    {
        return GetLimelightNTDouble(limelightName, "tid");
    }

    inline double GetNeuralClassID(const std::string &limelightName = "")
    {
        return GetLimelightNTDouble(limelightName, "tclass");
    }

    /////
    /////

    inline void SetPipelineIndex(const std::string &limelightName, int index)
    {
        SetLimelightNTDouble(limelightName, "pipeline", index);
    }

    inline void SetLEDMode_PipelineControl(const std::string &limelightName)
    {
        SetLimelightNTDouble(limelightName, "ledMode", 0);
    }

    inline void SetLEDMode_ForceOff(const std::string &limelightName)
    {
        SetLimelightNTDouble(limelightName, "ledMode", 1);
    }

    inline void SetLEDMode_ForceBlink(const std::string &limelightName)
    {
        SetLimelightNTDouble(limelightName, "ledMode", 2);
    }

    inline void SetLEDMode_ForceOn(const std::string &limelightName)
    {
        SetLimelightNTDouble(limelightName, "ledMode", 3);
    }

    inline void SetStreamMode_Standard(const std::string &limelightName)
    {
        SetLimelightNTDouble(limelightName, "stream", 0);
    }

    inline void SetStreamMode_PiPMain(const std::string &limelightName)
    {
        SetLimelightNTDouble(limelightName, "stream", 1);
    }

    inline void SetStreamMode_PiPSecondary(const std::string &limelightName)
    {
        SetLimelightNTDouble(limelightName, "stream", 2);
    }

    /**
     * Sets the crop window. The crop window in the UI must be completely open for
     * dynamic cropping to work.
     */
    inline void
    SetCropWindow(const std::string &limelightName, double cropXMin, double cropXMax, double cropYMin, double cropYMax)
    {
        double cropWindow[4]{ cropXMin, cropXMax, cropYMin, cropYMax };
        SetLimelightNTDoubleArray(limelightName, "crop", cropWindow);
    }

    /////
    /////

    inline void SetPythonScriptData(const std::string &limelightName, const std::vector<double> &outgoingPythonData)
    {
        SetLimelightNTDoubleArray(limelightName,
                                  "llrobot",
                                  std::span{ outgoingPythonData.begin(), outgoingPythonData.size() });
    }

    inline std::vector<double> GetPythonScriptData(const std::string &limelightName = "")
    {
        return GetLimelightNTDoubleArray(limelightName, "llpython");
    }

    /////
    /////

    // Take async snapshot

    /////
    /////

    inline const double INVALID_TARGET = 0.0;

    class SingleTargetingResultClass
    {
    public:
        SingleTargetingResultClass(){};
        ~SingleTargetingResultClass(){};
        double m_TargetXPixels{ INVALID_TARGET };
        double m_TargetYPixels{ INVALID_TARGET };

        double m_TargetXNormalized{ INVALID_TARGET };
        double m_TargetYNormalized{ INVALID_TARGET };

        double m_TargetXNormalizedCrosshairAdjusted{ INVALID_TARGET };
        double m_TargetYNormalizedCrosshairAdjusted{ INVALID_TARGET };

        double m_TargetXDegreesCrosshairAdjusted{ INVALID_TARGET };
        double m_TargetYDegreesCrosshairAdjusted{ INVALID_TARGET };

        double m_TargetAreaPixels{ INVALID_TARGET };
        double m_TargetAreaNormalized{ INVALID_TARGET };
        double m_TargetAreaNormalizedPercentage{ INVALID_TARGET };

        // not included in json//
        double m_timeStamp{ -1.0 };
        double m_TargetLatency{ 0 };
        double m_pipelineIndex{ -1.0 };
        std::vector<std::vector<double>> m_TargetCorners;

        std::vector<double> m_CAMERATransform6DTARGETSPACE;
        std::vector<double> m_TargetTransform6DCAMERASPACE;
        std::vector<double> m_TargetTransform6DROBOTSPACE;
        std::vector<double> m_ROBOTTransform6DTARGETSPACE;
        std::vector<double> m_ROBOTTransform6DFIELDSPACE;
    };

    class RetroreflectiveResultClass : public SingleTargetingResultClass
    {
    public:
        RetroreflectiveResultClass() {}

        ~RetroreflectiveResultClass() {}
    };

    class FiducialResultClass : public SingleTargetingResultClass
    {
    public:
        FiducialResultClass() {}

        ~FiducialResultClass() {}

        int m_fiducialID{ 0 };
        std::string m_family{ "" };
    };

    class DetectionResultClass : public SingleTargetingResultClass
    {
    public:
        DetectionResultClass() {}

        ~DetectionResultClass() {}

        int m_classID{ -1 };
        std::string m_className{ "" };
        double m_confidence{ 0 };
    };

    class ClassificationResultClass : public SingleTargetingResultClass
    {
    public:
        ClassificationResultClass() {}

        ~ClassificationResultClass() {}

        int m_classID{ -1 };
        std::string m_className{ "" };
        double m_confidence{ 0 };
    };

    class VisionResultsClass
    {
    public:
        VisionResultsClass() {}

        ~VisionResultsClass() {}

        std::vector<RetroreflectiveResultClass> RetroResults;
        std::vector<FiducialResultClass> FiducialResults;
        std::vector<DetectionResultClass> DetectionResults;
        std::vector<ClassificationResultClass> ClassificationResults;
        double m_timeStamp{ -1.0 };
        double m_TargetLatency{ 0 };
        double m_JsonParseLatency{ 0 };
        double m_pipelineIndex{ -1.0 };
        int valid{ 0 };
        std::vector<double> botPose;
        std::vector<double> botPose_wpired;
        std::vector<double> botPose_wpiblue;

        void Clear()
        {
            RetroResults.clear();
            FiducialResults.clear();
            DetectionResults.clear();
            ClassificationResults.clear();
            botPose.clear();
            botPose_wpired.clear();
            botPose_wpiblue.clear();
            m_timeStamp     = -1.0;
            m_TargetLatency = 0;
            m_pipelineIndex = -1.0;
        }
    };

    class LimelightResultsClass
    {
    public:
        LimelightResultsClass() {}

        ~LimelightResultsClass() {}

        VisionResultsClass targetingResults;
    };

    inline LimelightResultsClass GetLatestResults(const std::string &limelightName = "", bool profile = false)
    {
        auto start             = std::chrono::high_resolution_clock::now();
        std::string jsonString = GetJSONDump(limelightName);
        wpi::json data         = wpi::json::parse(jsonString);
        auto end               = std::chrono::high_resolution_clock::now();
        double nanos           = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        double millis          = (nanos * 0.000001);

        LimelightResultsClass out               = data.get<LimelightResultsClass>();
        out.targetingResults.m_JsonParseLatency = millis;
        if (profile)
        {
            std::cout << "lljson: " << millis << std::endl;
        }
        return out;
    }

    namespace internal
    {
        inline const std::string _key_timestamp{ "ts" };
        inline const std::string _key_latency{ "tl" };
        inline const std::string _key_pipelineIndex{ "pID" };
        inline const std::string _key_TargetXDegrees{ "txdr" };
        inline const std::string _key_TargetYDegrees{ "tydr" };
        inline const std::string _key_TargetXNormalized{ "txnr" };
        inline const std::string _key_TargetYNormalized{ "tynr" };
        inline const std::string _key_TargetXPixels{ "txp" };
        inline const std::string _key_TargetYPixels{ "typ" };

        inline const std::string _key_TargetXDegreesCrosshair{ "tx" };
        inline const std::string _key_TargetYDegreesCrosshair{ "ty" };
        inline const std::string _key_TargetXNormalizedCrosshair{ "txn" };
        inline const std::string _key_TargetYNormalizedCrosshair{ "tyn" };
        inline const std::string _key_TargetAreaNormalized{ "ta" };
        inline const std::string _key_TargetAreaPixels{ "tap" };
        inline const std::string _key_className{ "class" };
        inline const std::string _key_classID{ "classID" };
        inline const std::string _key_confidence{ "conf" };
        inline const std::string _key_fiducialID{ "fID" };
        inline const std::string _key_corners{ "pts" };
        inline const std::string _key_transformCAMERAPOSE_TARGETSPACE{ "t6c_ts" };
        inline const std::string _key_transformTARGETPOSE_CAMERASPACE{ "t6t_cs" };
        inline const std::string _key_transformROBOTPOSE_TARGETSPACE{ "t6r_ts" };
        inline const std::string _key_transformTARGETPOSE_ROBOTSPACE{ "t6t_rs" };

        inline const std::string _key_transformROBOTPOSE_FIELDSPACE{ "t6r_fs" };
        inline const std::string _key_skew{ "skew" };
        inline const std::string _key_ffamily{ "fam" };
        inline const std::string _key_colorRGB{ "cRGB" };
        inline const std::string _key_colorHSV{ "cHSV" };
    } // namespace internal

    inline void from_json(const wpi::json &j, RetroreflectiveResultClass &t)
    {
        t.m_CAMERATransform6DTARGETSPACE
            = j.at(internal::_key_transformCAMERAPOSE_TARGETSPACE).get<std::vector<double>>();
        t.m_TargetTransform6DCAMERASPACE
            = j.at(internal::_key_transformTARGETPOSE_CAMERASPACE).get<std::vector<double>>();
        t.m_TargetTransform6DROBOTSPACE
            = j.at(internal::_key_transformTARGETPOSE_ROBOTSPACE).get<std::vector<double>>();
        t.m_ROBOTTransform6DTARGETSPACE
            = j.at(internal::_key_transformROBOTPOSE_TARGETSPACE).get<std::vector<double>>();
        t.m_ROBOTTransform6DFIELDSPACE = j.at(internal::_key_transformROBOTPOSE_FIELDSPACE).get<std::vector<double>>();
        t.m_TargetXPixels              = j.at(internal::_key_TargetXPixels).get<double>();
        t.m_TargetYPixels              = j.at(internal::_key_TargetYPixels).get<double>();
        t.m_TargetXDegreesCrosshairAdjusted = j.at(internal::_key_TargetXDegreesCrosshair).get<double>();
        t.m_TargetYDegreesCrosshairAdjusted = j.at(internal::_key_TargetYDegreesCrosshair).get<double>();
        t.m_TargetAreaNormalized            = j.at(internal::_key_TargetAreaNormalized).get<double>();
        t.m_TargetCorners                   = j.at(internal::_key_corners).get<std::vector<std::vector<double>>>();
    }

    inline void from_json(const wpi::json &j, FiducialResultClass &t)
    {
        t.m_family     = j.at(internal::_key_ffamily).get<std::string>();
        t.m_fiducialID = j.at(internal::_key_fiducialID).get<double>();
        t.m_CAMERATransform6DTARGETSPACE
            = j.at(internal::_key_transformCAMERAPOSE_TARGETSPACE).get<std::vector<double>>();
        t.m_TargetTransform6DCAMERASPACE
            = j.at(internal::_key_transformTARGETPOSE_CAMERASPACE).get<std::vector<double>>();
        t.m_TargetTransform6DROBOTSPACE
            = j.at(internal::_key_transformTARGETPOSE_ROBOTSPACE).get<std::vector<double>>();
        t.m_ROBOTTransform6DTARGETSPACE
            = j.at(internal::_key_transformROBOTPOSE_TARGETSPACE).get<std::vector<double>>();
        t.m_ROBOTTransform6DFIELDSPACE = j.at(internal::_key_transformROBOTPOSE_FIELDSPACE).get<std::vector<double>>();
        t.m_TargetXPixels              = j.at(internal::_key_TargetXPixels).get<double>();
        t.m_TargetYPixels              = j.at(internal::_key_TargetYPixels).get<double>();
        t.m_TargetXDegreesCrosshairAdjusted = j.at(internal::_key_TargetXDegreesCrosshair).get<double>();
        t.m_TargetYDegreesCrosshairAdjusted = j.at(internal::_key_TargetYDegreesCrosshair).get<double>();
        t.m_TargetAreaNormalized            = j.at(internal::_key_TargetAreaNormalized).get<double>();
        t.m_TargetCorners                   = j.at(internal::_key_corners).get<std::vector<std::vector<double>>>();
    }

    inline void from_json(const wpi::json &j, DetectionResultClass &t)
    {
        t.m_confidence                      = j.at(internal::_key_confidence).get<double>();
        t.m_classID                         = j.at(internal::_key_classID).get<double>();
        t.m_className                       = j.at(internal::_key_className).get<std::string>();
        t.m_TargetXPixels                   = j.at(internal::_key_TargetXPixels).get<double>();
        t.m_TargetYPixels                   = j.at(internal::_key_TargetYPixels).get<double>();
        t.m_TargetXDegreesCrosshairAdjusted = j.at(internal::_key_TargetXDegreesCrosshair).get<double>();
        t.m_TargetYDegreesCrosshairAdjusted = j.at(internal::_key_TargetYDegreesCrosshair).get<double>();
        t.m_TargetAreaNormalized            = j.at(internal::_key_TargetAreaNormalized).get<double>();
        t.m_TargetCorners                   = j.at(internal::_key_corners).get<std::vector<std::vector<double>>>();
    }

    inline void from_json(const wpi::json &j, ClassificationResultClass &t)
    {
        t.m_confidence                      = j.at(internal::_key_confidence).get<double>();
        t.m_classID                         = j.at(internal::_key_classID).get<double>();
        t.m_className                       = j.at(internal::_key_className).get<std::string>();
        t.m_TargetXPixels                   = j.at(internal::_key_TargetXPixels).get<double>();
        t.m_TargetYPixels                   = j.at(internal::_key_TargetYPixels).get<double>();
        t.m_TargetXDegreesCrosshairAdjusted = j.at(internal::_key_TargetXDegreesCrosshair).get<double>();
        t.m_TargetYDegreesCrosshairAdjusted = j.at(internal::_key_TargetYDegreesCrosshair).get<double>();
        t.m_TargetAreaNormalized            = j.at(internal::_key_TargetAreaNormalized).get<double>();
        t.m_TargetCorners                   = j.at(internal::_key_corners).get<std::vector<std::vector<double>>>();
    }

    inline void from_json(const wpi::json &j, VisionResultsClass &t)
    {
        t.m_timeStamp     = j.at(internal::_key_timestamp).get<double>();
        t.m_TargetLatency = j.at(internal::_key_latency).get<double>();
        t.m_pipelineIndex = j.at(internal::_key_pipelineIndex).get<double>();
        t.valid           = j.at("v").get<double>();

        t.botPose         = j.at("botpose").get<std::vector<double>>();
        t.botPose_wpired  = j.at("botpose_wpired").get<std::vector<double>>();
        t.botPose_wpiblue = j.at("botpose_wpiblue").get<std::vector<double>>();

        t.RetroResults          = j.at("Retro").get<std::vector<RetroreflectiveResultClass>>();
        t.FiducialResults       = j.at("Fiducial").get<std::vector<FiducialResultClass>>();
        t.DetectionResults      = j.at("Detector").get<std::vector<DetectionResultClass>>();
        t.ClassificationResults = j.at("Detector").get<std::vector<ClassificationResultClass>>();
    }

    inline void from_json(const wpi::json &j, LimelightResultsClass &t)
    {
        t.targetingResults = j.at("Results").get<VisionResultsClass>();
    }
} // namespace LimelightHelpers

#endif // LIMELIGHTHELPERS_H
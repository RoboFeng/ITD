#include <IntensityTD.hpp>

ITDLog::ITDLog(ros::NodeHandle nh)
{
    nh.param<bool>("log_predict_score_en", log_predict_score_en, false);
    nh.param<string>("log_predict_score_path", log_predict_score_path, "/Log/pr.txt");

    nh.param<bool>("log_time_cost_en", log_time_cost_en, false);
    nh.param<string>("log_time_cost_path", log_time_cost_path, "/Log/timecost.txt");

    if (log_predict_score_en)
        fp_predict_score = fopen((ROOT_DIR + log_predict_score_path).c_str(), "w");

    if (log_time_cost_en)
        fp_time_cost = fopen((ROOT_DIR + log_time_cost_path).c_str(), "w");
}

// Save keyframe labels, timestamps, and relative transition matrices for calculating PR curves
void ITDLog::logDumpPredictScore(int id1, int id2, double t1, double t2, float s1, float s2)
{
    if (log_predict_score_en)
    {
        fprintf(fp_predict_score, "%d,", id1);
        fprintf(fp_predict_score, "%d,", id2);
        fprintf(fp_predict_score, "%f,", t1);
        fprintf(fp_predict_score, "%f,", t2);
        fprintf(fp_predict_score, "%f,", s1);
        fprintf(fp_predict_score, "%f", s2);

        fprintf(fp_predict_score, "\r\n");
        fflush(fp_predict_score);
    }
};

// Save the time consumption of each part of the algorithm
void ITDLog::logDumpTimeCost(double timestamp, double t1, double t2, double t3, double t4, double t5)
{
    if (log_time_cost_en)
    {
        fprintf(fp_time_cost, "%f,", timestamp);
        fprintf(fp_time_cost, "%f,", t1);
        fprintf(fp_time_cost, "%f,", t2);
        fprintf(fp_time_cost, "%f,", t3);
        fprintf(fp_time_cost, "%f,", t4);
        fprintf(fp_time_cost, "%f", t5);

        fprintf(fp_time_cost, "\r\n");
        fflush(fp_time_cost);
    }
};
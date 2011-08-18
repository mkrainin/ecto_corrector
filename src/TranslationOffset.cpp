/*
 * TranslationOffset.cpp
 *
 *  Created on: Aug 16, 2011
 *      Author: mkrainin
 */

#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>


namespace ecto_corrector
{
  using ecto::tendrils;

  struct TranslationOffset
  {
  public:
    static void declare_params(tendrils& params)
    {
      params.declare<double>("tx","x translation",0.0);
      params.declare<double>("ty","y translation",0.0);
      params.declare<double>("tz","z translation",0.0);
    }

    static void declare_io(const tendrils& /*params*/, tendrils& in, tendrils& out)
    {
      //inputs
      in.declare<cv::Mat>("R", "3x3 cv:Mat defining the rotation of the coordinate system");
      in.declare<cv::Mat>("T", "3x1 cv:Mat defining the original translation of the coordinate system");

      //outputs
      out.declare<cv::Mat>("out", "3x1 cv:Mat containing the new translation of the coordinate system after the offset is applied");
    }

    void configure(const tendrils& params, const tendrils& in, const tendrils& out)
    {
      //inputs
      t_ = in["T"];
      r_ = in["R"];

      //outputs
      out_ = out["out"];

      //params
      tx_ = params["tx"];
      ty_ = params["ty"];
      tz_ = params["tz"];
    }

    int process(const tendrils& in, const tendrils& out)
    {
      if(r_->empty() || t_->empty())
        return ecto::OK;

      cv::Mat offset(3,1,CV_64F);
      offset.at<double>(0,0) = *tx_;
      offset.at<double>(0,1) = *ty_;
      offset.at<double>(0,2) = *tz_;
      cv::gemm(*r_,offset,1,*t_,1,*out_);

      return ecto::OK;
    }

  private:
    //inputs
    ecto::spore<cv::Mat> t_,r_;

    //outputs
    ecto::spore<cv::Mat> out_;

    //params
    ecto::spore<double> tx_,ty_,tz_;

  };
} //namespace

ECTO_CELL(ecto_corrector, ecto_corrector::TranslationOffset, "TranslationOffset",
          "Translates by the specified amount in the coodinate axes defined by R");

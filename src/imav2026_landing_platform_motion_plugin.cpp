#include <algorithm>
#include <functional>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

namespace gazebo
{
class Imav2026LandingPlatformMotionPlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    this->model_ = model;
    if (!this->model_) {
      gzerr << "Imav2026LandingPlatformMotionPlugin: model pointer is null.\n";
      return;
    }

    if (sdf->HasElement("min_x")) {
      this->min_x_ = sdf->Get<double>("min_x");
    }
    if (sdf->HasElement("max_x")) {
      this->max_x_ = sdf->Get<double>("max_x");
    }
    if (sdf->HasElement("speed_mps")) {
      this->speed_mps_ = sdf->Get<double>("speed_mps");
    }

    if (this->min_x_ > this->max_x_) {
      std::swap(this->min_x_, this->max_x_);
    }

    this->speed_mps_ = std::max(0.0, this->speed_mps_);
    this->last_time_ = this->model_->GetWorld()->SimTime();

    const auto pose = this->model_->WorldPose();
    this->direction_ = (pose.Pos().X() <= this->min_x_) ? 1.0 : -1.0;

    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&Imav2026LandingPlatformMotionPlugin::OnUpdate, this));

    gzmsg << "Imav2026LandingPlatformMotionPlugin loaded for ["
          << this->model_->GetName() << "] min_x=" << this->min_x_
          << " max_x=" << this->max_x_
          << " speed_mps=" << this->speed_mps_ << "\n";
  }

private:
  void OnUpdate()
  {
    if (!this->model_ || this->speed_mps_ <= 0.0) {
      return;
    }

    const auto current_time = this->model_->GetWorld()->SimTime();
    const double dt = (current_time - this->last_time_).Double();
    this->last_time_ = current_time;

    if (dt <= 0.0) {
      return;
    }

    auto pose = this->model_->WorldPose();
    double next_x = pose.Pos().X() + this->direction_ * this->speed_mps_ * dt;

    if (next_x >= this->max_x_) {
      next_x = this->max_x_;
      this->direction_ = -1.0;
    } else if (next_x <= this->min_x_) {
      next_x = this->min_x_;
      this->direction_ = 1.0;
    }

    pose.Pos().X(next_x);
    this->model_->SetWorldPose(pose);
  }

  physics::ModelPtr model_;
  event::ConnectionPtr update_connection_;
  common::Time last_time_;
  double min_x_{1.569980};
  double max_x_{2.727315};
  double speed_mps_{0.2};
  double direction_{-1.0};
};

GZ_REGISTER_MODEL_PLUGIN(Imav2026LandingPlatformMotionPlugin)
} // namespace gazebo

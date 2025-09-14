#include <mc_rbdyn/Frame.h>

#include <mc_rbdyn/Robot.h>

#include <mc_tvm/Frame.h>

namespace mc_rbdyn
{

Frame::Frame(NewFrameToken, const std::string & name) noexcept : name_(name) {}

Frame::Frame(NewFrameToken tkn, const std::string & name, Frame & parent, sva::PTransformd X_p_f, bool bake) noexcept
: Frame(tkn, name)
{
  if(bake)
  {
    position_ = X_p_f * parent.position_;
    velocity_ = X_p_f * parent.velocity_;
  }
  else
  {
    parent_ = parent;
    position_ = X_p_f;
  }
}

Frame::~Frame() = default;

FramePtr Frame::make(const std::string & name, Frame & parent, sva::PTransformd X_p_f, bool baked)
{
  auto robot_frame = dynamic_cast<RobotFrame *>(&parent);
  if(robot_frame)
  {
    auto & robot = robot_frame->robot();
    return robot.makeFrame(name, *robot_frame, X_p_f, baked);
  }
  return std::make_shared<Frame>(NewFrameToken{}, name, parent, X_p_f, baked);
}

void Frame::init_tvm_frame() const
{
  tvm_frame_.reset(new mc_tvm::Frame(mc_tvm::Frame::NewFrameToken{}, *this));
}

} // namespace mc_rbdyn

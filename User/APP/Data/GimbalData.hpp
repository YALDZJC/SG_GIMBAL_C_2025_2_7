

namespace APP::Data
{

class GimbalData
{
  private:
    float tar_yaw;
    float tar_pitch;

  public:
    void setTarYaw(float yaw)
    {
        tar_yaw = yaw;
    }

    void setTarPitch(float pitch)
    {
        tar_pitch = pitch;
    }

    float getTarYaw()
    {
        return tar_yaw;
    }

    float getTarPitch()
    {
        return tar_pitch;
    }
};

inline GimbalData gimbal_data;
} // namespace APP::Gimbal
// hal_core.h
namespace HAL
{
class Core
{
  public:
    static Core *getInstance();
    static bool inject(Core *instance);
    static void destroy();

    virtual ~Core() = default;
    virtual bool initialize() = 0; // 总初始化接口
};
} // namespace HAL
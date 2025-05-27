#include "costmaps_only/costmaps_server.hpp"
#include "tf2_ros/transform_listener.h"

CostmapsServer::CostmapsServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("costmaps_server", "", options)
{
  // Принудительно устанавливаем уровень логирования для DEBUG
  auto logger = this->get_logger();
  rcutils_logging_set_logger_level(logger.get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

  RCLCPP_INFO(logger, "[ИНИЦИАЛИЗАЦИЯ] Создаем сервер costmaps");
  RCLCPP_DEBUG(logger, "Проверяем параметр use_sim_time...");
  
  bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
  RCLCPP_DEBUG(logger, "Параметр use_sim_time: %s", use_sim_time ? "true" : "false");

  RCLCPP_DEBUG(logger, "Инициализируем глобальную costmap...");
  global_costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "global_costmap", get_namespace(), "global_costmap", use_sim_time);
  
  RCLCPP_DEBUG(logger, "Инициализируем локальную costmap...");
  local_costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "local_costmap", get_namespace(), "local_costmap", use_sim_time);

  logCostmapInfo(global_costmap_, "Глобальная");
  logCostmapInfo(local_costmap_, "Локальная");

  RCLCPP_DEBUG(logger, "Создаем executor для costmaps...");
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor_->add_node(global_costmap_->get_node_base_interface());
  executor_->add_node(local_costmap_->get_node_base_interface());
  
  RCLCPP_DEBUG(logger, "Запускаем потоки для costmaps...");
  costmap_thread_ = std::make_unique<std::thread>([this, logger]() {
    RCLCPP_INFO(logger, "[ПОТОК] Запущен executor для costmaps");
    try {
      executor_->spin();
      RCLCPP_INFO(logger, "[ПОТОК] Executor завершил работу нормально");
    } catch (const std::exception & e) {
      RCLCPP_FATAL(logger, "[ПОТОК] Ошибка в executor: %s", e.what());
    }
  });
}

CostmapsServer::~CostmapsServer()
{
  auto logger = this->get_logger();
  RCLCPP_DEBUG(logger, "[УНИЧТОЖЕНИЕ] Начинаем завершение работы сервера");
  
  if (executor_) {
    RCLCPP_DEBUG(logger, "Останавливаем executor...");
    executor_->cancel();
  }
  
  if (costmap_thread_ && costmap_thread_->joinable()) {
    RCLCPP_DEBUG(logger, "Ожидаем завершения потока executor...");
    costmap_thread_->join();
    RCLCPP_DEBUG(logger, "Поток executor завершен");
  }
  
  global_costmap_.reset();
  local_costmap_.reset();
  RCLCPP_INFO(logger, "[УНИЧТОЖЕНИЕ] Сервер costmaps полностью завершил работу");
}

void CostmapsServer::logCostmapInfo(
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap, 
  const std::string & name)
{
  auto logger = this->get_logger();
  RCLCPP_DEBUG(logger, "--- %s Costmap Информация ---", name.c_str());
  RCLCPP_DEBUG(logger, "  Глобальный фрейм: %s", costmap->getGlobalFrameID().c_str());
  RCLCPP_DEBUG(logger, "  Базовый фрейм: %s", costmap->getBaseFrameID().c_str());
  RCLCPP_DEBUG(logger, "  Текущее состояние: %s", costmap->get_current_state().label().c_str());
}

bool CostmapsServer::checkTransform(
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string & target_frame,
  const std::string & source_frame,
  const rclcpp::Duration & timeout)
{
  auto logger = this->get_logger();
  RCLCPP_DEBUG(logger, "Проверяем трансформацию из '%s' в '%s' (таймаут: %.2fs)", 
              source_frame.c_str(), target_frame.c_str(), timeout.seconds());
  
  try {
    if (tf_buffer->canTransform(target_frame, source_frame, tf2::TimePointZero, 
                               tf2::durationFromSec(timeout.seconds()))) {
      RCLCPP_DEBUG(logger, "Трансформация доступна");
      return true;
    }
    RCLCPP_WARN(logger, "Трансформация не доступна");
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(logger, "Ошибка трансформации: %s", ex.what());
  }
  return false;
}

nav2_util::CallbackReturn CostmapsServer::on_configure(const rclcpp_lifecycle::State & state)
{
  auto logger = this->get_logger();
  RCLCPP_INFO(logger, "[КОНФИГУРАЦИЯ] Начинаем настройку costmaps");
  
  // Конфигурация глобальной costmap
  RCLCPP_DEBUG(logger, "Настраиваем глобальную costmap...");
  auto global_ret = global_costmap_->on_configure(state);
  if (global_ret != nav2_util::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(logger, "Ошибка настройки глобальной costmap (состояние: %s)", 
                global_costmap_->get_current_state().label().c_str());
    return nav2_util::CallbackReturn::FAILURE;
  }
  logCostmapInfo(global_costmap_, "Глобальная (настроена)");

  // Конфигурация локальной costmap
  RCLCPP_DEBUG(logger, "Настраиваем локальную costmap...");
  auto local_ret = local_costmap_->on_configure(state);
  if (local_ret != nav2_util::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(logger, "Ошибка настройки локальной costmap (состояние: %s)", 
                local_costmap_->get_current_state().label().c_str());
    global_costmap_->on_cleanup(state);
    return nav2_util::CallbackReturn::FAILURE;
  }
  logCostmapInfo(local_costmap_, "Локальная (настроена)");

  RCLCPP_INFO(logger, "[КОНФИГУРАЦИЯ] Costmaps успешно настроены");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn CostmapsServer::on_activate(const rclcpp_lifecycle::State & state)
{
  auto logger = this->get_logger();
  RCLCPP_INFO(logger, "[АКТИВАЦИЯ] Активируем costmaps");
  
  // Активация глобальной costmap
  RCLCPP_DEBUG(logger, "Проверяем трансформации для глобальной costmap...");
  if (!checkTransform(global_costmap_->getTfBuffer(),
                     global_costmap_->getGlobalFrameID(),
                     global_costmap_->getBaseFrameID(),
                     rclcpp::Duration::from_seconds(5.0))) {
    RCLCPP_ERROR(logger, "Нет доступных трансформаций для глобальной costmap");
    return nav2_util::CallbackReturn::FAILURE;
  }

  RCLCPP_DEBUG(logger, "Активируем глобальную costmap...");
  auto global_ret = global_costmap_->on_activate(state);
  if (global_ret != nav2_util::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(logger, "Ошибка активации глобальной costmap (состояние: %s)", 
                global_costmap_->get_current_state().label().c_str());
    return nav2_util::CallbackReturn::FAILURE;
  }
  logCostmapInfo(global_costmap_, "Глобальная (активирована)");

  // Активация локальной costmap
  RCLCPP_DEBUG(logger, "Проверяем трансформации для локальной costmap...");
  if (!checkTransform(local_costmap_->getTfBuffer(),
                     local_costmap_->getGlobalFrameID(),
                     local_costmap_->getBaseFrameID(),
                     rclcpp::Duration::from_seconds(5.0))) {
    RCLCPP_ERROR(logger, "Нет доступных трансформаций для локальной costmap");
    global_costmap_->on_deactivate(state);
    return nav2_util::CallbackReturn::FAILURE;
  }

  RCLCPP_DEBUG(logger, "Активируем локальную costmap...");
  auto local_ret = local_costmap_->on_activate(state);
  if (local_ret != nav2_util::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(logger, "Ошибка активации локальной costmap (состояние: %s)", 
                local_costmap_->get_current_state().label().c_str());
    global_costmap_->on_deactivate(state);
    return nav2_util::CallbackReturn::FAILURE;
  }
  logCostmapInfo(local_costmap_, "Локальная (активирована)");

  // Создание bond соединения
  RCLCPP_DEBUG(logger, "Устанавливаем bond соединение...");
  createBond();
  
  RCLCPP_INFO(logger, "[АКТИВАЦИЯ] Costmaps успешно активированы");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn CostmapsServer::on_deactivate(const rclcpp_lifecycle::State & state)
{
  auto logger = this->get_logger();
  RCLCPP_INFO(logger, "[ДЕАКТИВАЦИЯ] Деактивируем costmaps");
  
  RCLCPP_DEBUG(logger, "Разрываем bond соединение...");
  destroyBond();

  // Деактивация costmap'ов
  RCLCPP_DEBUG(logger, "Деактивируем costmaps...");
  local_costmap_->on_deactivate(state);
  global_costmap_->on_deactivate(state);

  logCostmapInfo(global_costmap_, "Глобальная (деактивирована)");
  logCostmapInfo(local_costmap_, "Локальная (деактивирована)");

  RCLCPP_INFO(logger, "[ДЕАКТИВАЦИЯ] Costmaps успешно деактивированы");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn CostmapsServer::on_cleanup(const rclcpp_lifecycle::State & state)
{
  auto logger = this->get_logger();
  RCLCPP_INFO(logger, "[ОЧИСТКА] Очищаем costmaps");
  
  // Очистка costmap'ов
  RCLCPP_DEBUG(logger, "Очищаем costmaps...");
  local_costmap_->on_cleanup(state);
  global_costmap_->on_cleanup(state);

  logCostmapInfo(global_costmap_, "Глобальная (очищена)");
  logCostmapInfo(local_costmap_, "Локальная (очищена)");

  RCLCPP_INFO(logger, "[ОЧИСТКА] Costmaps успешно очищены");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn CostmapsServer::on_shutdown(const rclcpp_lifecycle::State & state)
{
  auto logger = this->get_logger();
  RCLCPP_INFO(logger, "[ВЫКЛЮЧЕНИЕ] Выключаем costmaps (причина: %s)", state.label().c_str());
  
  if (state.label() == "shutdown") {
    RCLCPP_DEBUG(logger, "Выполняем полное выключение costmaps...");
    local_costmap_->on_shutdown(state);
    global_costmap_->on_shutdown(state);
  }
  
  RCLCPP_INFO(logger, "[ВЫКЛЮЧЕНИЕ] Costmaps успешно выключены");
  return nav2_util::CallbackReturn::SUCCESS;
}

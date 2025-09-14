/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

/*! Interface used to load robots */

#include <mc_observers/Observer.h>
#include <mc_observers/api.h>
#include <mc_rtc/config.h>
#include <mc_rtc/loader.h>

#include <memory>
#include <mutex>

namespace mc_observers
{

/*! \class ObserverLoader
 * \brief Load Observer instances from shared libraries
 */
struct MC_OBSERVERS_DLLAPI ObserverLoader
{
public:
  /** Return an Observer constructed with the provided Args
   * \param name The module name
   * \param args The arguments provided to the module creation function
   * \note
   * It is the responsability of the caller to make sure that the signature of the module creation fits that declared by
   * the module
   */
  template<typename... Args>
  static std::shared_ptr<mc_observers::Observer> get_observer(const std::string & name, const Args &... args)
  {
    std::lock_guard<std::mutex> guard{mtx};
    init();
    return observer_loader->create_object(name, args...);
  }

  /** Returns an Observer runtime directory
   *
   * Empty when the module does not exist or when it's registered statically
   *
   * \param name Module name
   *
   */
  inline static std::string get_observer_runtime_directory(const std::string & name) noexcept
  {
    std::lock_guard<std::mutex> guard{mtx};
    init();
    return observer_loader->get_object_runtime_directory(name);
  }

  template<typename RetT, typename... Args>
  static void register_object(const std::string & name, std::function<RetT *(const Args &...)> callback)
  {
    std::unique_lock<std::mutex> guard{mtx};
    init();
    observer_loader->register_object(name, callback);
  }

  /** Add additional directories to the robot module path
   * \param paths Directories to be added to the module path
   */
  static inline void update_module_path(const std::vector<std::string> & paths)
  {
    std::lock_guard<std::mutex> guard{mtx};
    init();
    observer_loader->load_libraries(paths);
  }

  /** Remove all loaded libraries */
  static inline void clear()
  {
    std::lock_guard<std::mutex> guard{mtx};
    init(true);
    observer_loader->clear();
  }

  /** Check if a robot is available
   * \param name Robot name
   */
  static bool has_observer(const std::string & name)
  {
    std::lock_guard<std::mutex> guard{mtx};
    init();
    return observer_loader->has_object(name);
  }

  static void set_verbosity(bool verbose)
  {
    std::lock_guard<std::mutex> guard{mtx};
    verbose_ = verbose;
    if(observer_loader) { observer_loader->set_verbosity(verbose); }
  }

  /** Returns a list of available robots */
  static std::vector<std::string> available_observers()
  {
    std::lock_guard<std::mutex> guard{mtx};
    init();
    return observer_loader->objects();
  }

private:
  static inline void init(bool skip_default_path = false)
  {
    if(!observer_loader)
    {
      try
      {
        std::vector<std::string> default_path = {};
        if(!skip_default_path) { default_path.push_back(mc_rtc::MC_OBSERVERS_INSTALL_PREFIX); }
        observer_loader.reset(
            new mc_rtc::ObjectLoader<mc_observers::Observer>("MC_RTC_OBSERVER_MODULE", default_path, verbose_));
      }
      catch(const mc_rtc::LoaderException & exc)
      {
        mc_rtc::log::error("Failed to initialize ObserverLoader: {}", exc.what());
        throw(exc);
      }
    }
  }
  static std::unique_ptr<mc_rtc::ObjectLoader<mc_observers::Observer>> observer_loader;
  static bool verbose_;
  static std::mutex mtx;
};

} // namespace mc_observers

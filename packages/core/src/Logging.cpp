/*
 * Copyright (C) 2008 Robotics at Maryland
 * Copyright (C) 2008 Joseph Lisee <jlisee@umd.edu>
 * All rights reserved.
 *
 * Author: Joseph Lisee <jlisee@umd.edu>
 * File:  packages/core/src/Logging.cpp
 */

// STD Includes
#include <iostream>
#include <vector>

// Library Includes
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <log4cpp/Appender.hh>
#include <log4cpp/FileAppender.hh>
#include <log4cpp/OstreamAppender.hh>
#include <log4cpp/BasicLayout.hh>
#include <log4cpp/SimpleLayout.hh>
#include <log4cpp/PatternLayout.hh>
#include <log4cpp/Category.hh>

// Project Includes
#include "core/include/Logging.h"

namespace fs = boost::filesystem;

namespace ram {
namespace core {

Logging::Logging(core::ConfigNode config, core::SubsystemList deps) :
    Subsystem(config["name"].asString("VisionSystem"), deps)
{

    std::map<std::string, log4cpp::Appender*> appenders;
    
    if (config.exists("Appenders"))
    {
        // Create Appenders
        ConfigNode appenderConfig(config["Categories"]);
        NodeNameList subnodes = appenderConfig.subNodes();
        BOOST_FOREACH(std::string appenderName, subnodes)
        {
            ConfigNode config(appenderConfig[appenderName]);
            createAppender(appenderName, config, appenders);
        }

        // Create categories
        if (config.exists("Categories"))
        {
            ConfigNode categoryConfig(config["Categories"]);
            subnodes = categoryConfig.subNodes();

            // Go through each category
            BOOST_FOREACH(std::string categoryName, subnodes)
            {
                createCategory(categoryName, categoryConfig[categoryName],
                               appenders);
            }
        }
    }
}

Logging::~Logging()
{
    // Clean up al appenders
    std::pair<log4cpp::Appender*, std::vector<std::string> > pair;
    BOOST_FOREACH(pair, m_appenders)
    {
        log4cpp::Appender* appender = pair.first;
        // Remove from all categories
        BOOST_FOREACH(std::string categoryName, pair.second)
        {
            log4cpp::Category::getInstance(categoryName).
                removeAppender(appender);
        }

        // Free the appender
        delete appender;
    }
}

boost::filesystem::path Logging::getLogDir()
{
    // Gets the current time only once
    static boost::posix_time::ptime
        now(boost::posix_time::second_clock::local_time());

    // The startup time as a string
    std::string dateName(boost::posix_time::to_iso_extended_string(now));

    // The root of source directory
    boost::filesystem::path root(getenv("RAM_SVN_DIR"));

    // The resulting path
    return root / "logs" / dateName;
}

void Logging::createCategory(
    std::string name,
    ConfigNode config,
    std::map<std::string, log4cpp::Appender*>& appenders)
{
    // Create Category
    log4cpp::Category::getInstance(name);
    log4cpp::Category* category = log4cpp::Category::exists(name);

    if (config.exists("Appenders"))
    {
        ConfigNode appenderList(config["appenders"]);
        int appenderCount = appenderList.size();

        // Go through each appender name and attemp to add it to config
        for (int i = 0; i < appenderCount; ++i)
        {
            std::string appenderName(appenderList[i].asString());
            std::map<std::string, log4cpp::Appender*>::iterator iter =
                appenders.find(appenderName);

            if (iter != appenders.end())
            {
                // Add appender but don't shift ownership
                category->addAppender(*(iter->second));
            }
            else
            {
                std::cerr << "ERROR: no such appender '" << appenderName
                          << "'" << std::endl;
            }
        }

        // Build string to priority map
        std::map<std::string, log4cpp::Priority::PriorityLevel> nameToPriority;
        nameToPriority["emergency"] = log4cpp::Priority::EMERG;
        nameToPriority["emerg"] = log4cpp::Priority::EMERG;
        nameToPriority["fatal"] = log4cpp::Priority::FATAL;
        nameToPriority["alert"] = log4cpp::Priority::ALERT;
        nameToPriority["crit"] = log4cpp::Priority::CRIT;
        nameToPriority["critical"] = log4cpp::Priority::CRIT;
        nameToPriority["error"] = log4cpp::Priority::ERROR;
        nameToPriority["warn"] = log4cpp::Priority::WARN;
        nameToPriority["warning"] = log4cpp::Priority::WARN;
        nameToPriority["info"] = log4cpp::Priority::INFO;
        nameToPriority["debug"] = log4cpp::Priority::DEBUG;
        
        // Set priority
        std::string priorityName = config["priority"].asString("");
        boost::to_lower(priorityName);
        std::map<std::string, log4cpp::Priority::PriorityLevel>::iterator iter =
            nameToPriority.find(priorityName);
        if (nameToPriority.end() != iter)
        {
            category->setPriority(iter->second);
        }
        else
        {
            std::cerr << "WARNING: priority: '" << priorityName
                      << "' is not valid" << std::endl;
            category->setPriority(log4cpp::Priority::NOTSET);
        }
    }
    else
    {
        std::cerr << "WARNING: Category: '" << name << "' has no appenders "
                  << "all logging messages will be suppressed" << std::endl;
    }
}

log4cpp::Appender* Logging::createAppender(
    std::string name,
    ConfigNode config,
    std::map<std::string, log4cpp::Appender*>& appenders)
{
    std::string type = config["type"].asString();
    log4cpp::Appender* appender = 0;

    // Created based on given type (Not a factory but will do for now)
    if (type == "Console")
    {
        appender = new log4cpp::OstreamAppender(name, &std::cout);
    }
    else if (type == "File")
    {
        // Make sure log directory exists
        fs::path path(getLogDir());
        if (!fs::exists(path))
            fs::create_directory(path);
            
        // Determine file path
        std::string fileName = config["fileName"].asString(name);
        std::string filePath = (getLogDir() / fileName).string();
        
        // Create the appender which logs to the given file
        appender = new log4cpp::FileAppender(name, filePath, false);
    }
    else
    {
        std::cerr << "ERROR: invalid appender type: '" << type << "'"
                  << std::cout;
    }

    if (appender)
    {
        // Set layout (using default if needed)
        log4cpp::Layout* layout = createLayout(config["Layout"]);
        if (!layout)
            layout = new log4cpp::BasicLayout;
        appender->setLayout(layout);
        
        // Store the appender
        appenders[name] = appender;
        m_appenders[appender] = std::vector<std::string>();
    }

    return appender;
}
    
log4cpp::Layout* Logging::createLayout(ConfigNode config)
{
    std::string type = config["type"].asString();
    log4cpp::Layout* layout = 0;

    // Created based on given type (Not a factory but will do for now)
    if (type == "Pattern")
    {
        log4cpp::PatternLayout* patternLayout = new log4cpp::PatternLayout();
        patternLayout->setConversionPattern(
            config["pattern"].asString(
                log4cpp::PatternLayout::DEFAULT_CONVERSION_PATTERN));
    }
    else if (type == "Simple")
    {
        layout = new log4cpp::SimpleLayout();
    }
    else
    {
        std::cerr << "ERROR: invalid layout type: '" << type << "'"
                  << std::cout;
    }

    return layout;
}
    
} // namespace core
} // namespace ram

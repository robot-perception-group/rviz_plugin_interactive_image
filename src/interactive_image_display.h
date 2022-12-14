/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/*
* Modifications © University Stuttgart 2022 - Institute for Flight Dynamics and Control (IFR)
* Maintainer: Eric Price
*/

#ifndef INTERACTIVE_IMAGE_DISPLAY_H
#define INTERACTIVE_IMAGE_DISPLAY_H

#ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <ros/ros.h>
#include <QObject>

#include <OgreMaterial.h>
#include <OgreRenderTargetListener.h>
#include <OgreSharedPtr.h>

#include "rviz/image/image_display_base.h"
#include "rviz/image/ros_image_texture.h"
#include "rviz/render_panel.h"

#include "rviz/properties/bool_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#endif


namespace Ogre
{
class SceneNode;
class Rectangle2D;
} // namespace Ogre

namespace rviz_plugin_interactive_image
{

class RenderPanel : public  rviz::RenderPanel
{
Q_OBJECT
public:
  virtual void mousePressEvent(QMouseEvent* event);
  virtual void mouseReleaseEvent(QMouseEvent* event);
  virtual void mouseDoubleClickEvent(QMouseEvent* event);
  virtual void mouseMoveEvent(QMouseEvent* event);
  Q_SIGNALS:
  void mousePosition( int x, int y, Qt::MouseButtons buttons, int type );

};

/**
 * \class InteractiveImageDisplay
 *
 */
class InteractiveImageDisplay : public rviz::ImageDisplayBase
{
  Q_OBJECT
public:
  InteractiveImageDisplay();
  ~InteractiveImageDisplay() override;

  // Overrides from Display
  void onInitialize() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

public Q_SLOTS:
  virtual void updateNormalizeOptions();
  void gotInteraction( int x, int y, Qt::MouseButtons buttons, int type);
  virtual void updateSendTopic();

protected:
  // overrides from Display
  void onEnable() override;
  void onDisable() override;

  /* This is called by incomingMessage(). */
  void processMessage(const sensor_msgs::Image::ConstPtr& msg) override;

  Ogre::SceneManager* img_scene_manager_;

  rviz::ROSImageTexture texture_;

  //rviz::RenderPanel* render_panel_;
  RenderPanel* render_panel_;

  Ogre::SceneNode* img_scene_node_;
  Ogre::Rectangle2D* screen_rect_;
  Ogre::MaterialPtr material_;

  rviz::BoolProperty* normalize_property_;
  rviz::FloatProperty* min_property_;
  rviz::FloatProperty* max_property_;
  rviz::IntProperty* median_buffer_size_property_;

  rviz::RosTopicProperty* publish_topic_property_;
  rviz::BoolProperty* react_click_property_;
  rviz::BoolProperty* react_release_property_;
  rviz::BoolProperty* react_dblclk_property_;
  rviz::BoolProperty* react_move_property_;
  bool got_float_image_;

  QString output_topic_;
  ros::Publisher output_publisher_;

  // The ROS node handle.
  ros::NodeHandle nh_;

};

} // namespace rviz_plugin_interactive_image

#endif

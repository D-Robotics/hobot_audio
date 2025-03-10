// Copyright (c) 2024，D-Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "audio_engine/audioengine.h"

#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace hobot {
namespace audio {

void VoipDataCallback(const void *cookie, const HrscCallbackData *data) {
  AudioEngine::Instance()->VoipDataCallback_(cookie, data);
}

void WakeupDataCallback(const void *cookie, const HrscCallbackData *data,
                        const int keyword_index) {
  AudioEngine::Instance()->WakeupDataCallback_(cookie, data, keyword_index);
}

void AsrDataCallback(const void *cookie, const HrscCallbackData *data) {
  AudioEngine::Instance()->AsrDataCallback_(cookie, data);
}

void EventCallback(const void *cookie, HrscEventCallbackData event) {
  AudioEngine::Instance()->EventCallback_(cookie, event);
}

void CmdDataCallback(const void *cookie, const char *cmd) {
  AudioEngine::Instance()->CmdDataCallback_(cookie, cmd);
}

void DoaCallback(const void *cookie, int doa) {
  AudioEngine::Instance()->DoaCallback_(cookie, doa);
}

void AsrCallback(const void *cookie, const char *asr) {
  AudioEngine::Instance()->AsrCallback_(cookie, asr);
}


AudioEngine::AudioEngine() {}

AudioEngine::~AudioEngine() {
  if (adapter_buffer_) delete[] adapter_buffer_;
}

int AudioEngine::Init(AudioDataFunc audio_cb, AudioSmartDataFunc audio_smart_cb,
                      AudioCmdDataFunc cmd_cb, AudioEventFunc event_cb,
                      AudioASRFunc asr_cb,
                      AudioASRDataFunc asr_data_cb,
                      const int mic_chn, const std::string config_path,
                      const int voip_mode, const int mic_type,
                      const int asr_output_mode, const int asr_output_channel) {
  if (init_) {
    RCLCPP_WARN(rclcpp::get_logger("audio_capture"),
                "has already initialized.");
    return 0;
  }
  mic_chn_num_ = mic_chn;
  voip_mode_ = voip_mode;
  mic_type_ = mic_type;
  asr_mode_ = asr_output_mode;
  asr_channel_ = asr_output_channel;
  sdk_file_path_ = config_path + "/hrsc";
  if (sdkin_chn_num_ > mic_chn_num_) {
    sdkin_chn_num_ = 3;
  }
  int ret = InitSDK();
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("audio_capture"), "init audio sdk fail");
    return ret;
  }

  audio_cb_ = audio_cb;
  audio_smart_cb_ = audio_smart_cb;
  audio_cmd_cb_ = cmd_cb;
  audio_event_cb_ = event_cb;
  audio_asr_cb_ = asr_cb;
  audio_asr_data_cb_ = asr_data_cb;
  init_ = true;
  RCLCPP_WARN(rclcpp::get_logger("audio_capture"), "init hrsc sdk success!");
  return 0;
}

int AudioEngine::DeInit() { return 0; }

int AudioEngine::Start() {
  if (!init_) {
    RCLCPP_ERROR(rclcpp::get_logger("audio_capture"), "engine not init.");
    return -1;
  }

  if (start_) {
    RCLCPP_ERROR(rclcpp::get_logger("audio_capture"),
                 "engine has already started.");
    return 0;
  }

  start_ = true;
  if (save_file_) {
    audio_inconvert_file_.open(
        "./audio_sdk.pcm", std::ios::app | std::ios::out | std::ios::binary);
  }

  RCLCPP_WARN(rclcpp::get_logger("audio_capture"), "audioengine start success");
  return 0;
}

int AudioEngine::Reset() { return 0; }

int AudioEngine::Stop() {
  if (!start_) return 0;
  DeInitSDK();
  if (audio_inconvert_file_.is_open()) audio_inconvert_file_.close();
  return 0;
}

int AudioEngine::InitSDK() {
  input_cfg_.audio_channels = sdkin_chn_num_;
  RCLCPP_WARN(rclcpp::get_logger("audio_capture"), "audioengine sdk in chn:%d",
              sdkin_chn_num_);
  input_cfg_.sample_rate = 16000;
  input_cfg_.audio_format = kHrscAudioFormatPcm16Bit;

  output_cfg_.audio_channels = 1;
  output_cfg_.sample_rate = 16000;
  output_cfg_.audio_format = kHrscAudioFormatPcm16Bit;
  memset(&effect_cfg_, 0, sizeof(HrscEffectConfig));
  effect_cfg_.input_cfg = input_cfg_;
  effect_cfg_.output_cfg = output_cfg_;
  effect_cfg_.priv = &effect_cfg_;
  effect_cfg_.asr_timeout = 5000;
  effect_cfg_.cfg_file_path = sdk_file_path_.c_str();
  if (!voip_mode_) {
    std::string cus_path = sdk_file_path_ + "/cmd_word.json";
    RCLCPP_INFO(rclcpp::get_logger("audio_capture"),
                "hrsc sdk file path:%s, cmd file:%s", sdk_file_path_.c_str(),
                cus_path.c_str());
    if (cus_path.empty()) {
      effect_cfg_.custom_wakeup_word = nullptr;
    } else {
      std::fstream stream(cus_path, std::ios::in);
      if (!stream.is_open()) {
        effect_cfg_.custom_wakeup_word = nullptr;
      } else {
        // get length of file:
        stream.seekg(0, stream.end);
        int length = stream.tellg();
        stream.seekg(0, stream.beg);
        char *buffer = new char[length + 1];
        stream.read(buffer, length);
        buffer[length] = '\0';
        effect_cfg_.custom_wakeup_word = buffer;
        stream.close();
        // delete[] buffer;
        std::cout << "hrsc sdk wakeup word is:" << std::endl
                  << effect_cfg_.custom_wakeup_word << std::endl;
      }
    }
  } else {
    effect_cfg_.custom_wakeup_word = nullptr;
  }

  effect_cfg_.vad_timeout = 5000;
  effect_cfg_.ref_ch_index = 6;  // 参考信号通道index
  effect_cfg_.target_score = 0;
  effect_cfg_.support_command_word = 0;
  effect_cfg_.wakeup_prefix = 200;
  effect_cfg_.wakeup_suffix = 200;
  effect_cfg_.is_use_linear_mic_flag = mic_type_;

  effect_cfg_.asr_output_mode = asr_mode_;
  effect_cfg_.asr_output_channel = asr_channel_;

  effect_cfg_.HrscVoipDataCallback = VoipDataCallback;
  effect_cfg_.HrscWakeupDataCallback = WakeupDataCallback;
  effect_cfg_.HrscAsrDataCallback = AsrDataCallback;
  effect_cfg_.HrscEventCallback = EventCallback;
  effect_cfg_.HrscCmdCallback = CmdDataCallback;
  effect_cfg_.HrscDoaCallbadk = DoaCallback;
  effect_cfg_.HrscAsrCallback = AsrCallback;

  sdk_handle_ = HrscInit(&effect_cfg_);
  if (sdk_handle_ == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("audio_capture"), "hrsc init error!!!");
    return -1;
  }
  RCLCPP_WARN(rclcpp::get_logger("audio_capture"), "hrsc init success!");
  if (HRSC_CODE_SUCCESS != HrscStart(sdk_handle_)) {
    RCLCPP_ERROR(rclcpp::get_logger("audio_capture"), "hrsc start error!!!");
    return -1;
  }

  int value = 0;
  if (voip_mode_) {
    RCLCPP_WARN(rclcpp::get_logger("audio_capture"),
                "audioengine sdk init on voip mode");
    value = 1;
  }
  HrscParamData data;
  data.value = &value;
  data.param_type = kHrscParasTypeVoipDataSwitch;
  HrscSetParam(sdk_handle_, &data);
  RCLCPP_WARN(rclcpp::get_logger("audio_capture"), "hrsc start success!");
  return 0;
}

int AudioEngine::InputData(char *data, int len, bool end) {
  if (!init_ || !start_) {
    RCLCPP_ERROR(rclcpp::get_logger("audio_capture"),
                 "engine not init or start.");
    return -1;
  }
  RCLCPP_DEBUG(rclcpp::get_logger("audio_capture"),
              "audioengine send audio data to sdk");
  if (end) {
    RCLCPP_DEBUG(rclcpp::get_logger("audio_capture"),
              "audioengine recv the last audio frame");
  }
  int size = len / mic_chn_num_ * sdkin_chn_num_;
  if (audio_size_ != size) {
    if (adapter_buffer_) delete[] adapter_buffer_;
    audio_size_ = size;
    adapter_buffer_ = new char[size];
    memset(adapter_buffer_, 0, size);
  }

  if (mic_chn_num_ == sdkin_chn_num_) {
    memcpy(adapter_buffer_, data, size);
  } else if (mic_chn_num_ < sdkin_chn_num_) {
    // 2通道数据扩展为3通道数据
    char *dst_ptr = adapter_buffer_;
    char *src_ptr = data;
    int frame_count = len / (mic_chn_num_ * 2);
    int index = 0;
    while (index++ < frame_count) {
      memcpy(dst_ptr, src_ptr, mic_chn_num_ * 2);
      dst_ptr += sdkin_chn_num_ * 2;
      src_ptr += mic_chn_num_ * 2;
    }
  } else {
    // 8通道数据->6通道数据,剔除5, 6通道
    char *dst_ptr = adapter_buffer_;
    char *src_ptr = data;
    int frame_count = len / (mic_chn_num_ * 2);
    int index = 0;
    while (index++ < frame_count) {
      memcpy(dst_ptr, src_ptr, 4 * 2);
      dst_ptr += 4 * 2;
      src_ptr += 6 * 2;
      memcpy(dst_ptr, src_ptr, 2 * 2);
      dst_ptr += 2 * 2;
      src_ptr += 2 * 2;
    }
  }

  HrscAudioBuffer hrsc_buffer;
  hrsc_buffer.audio_data = adapter_buffer_;
  hrsc_buffer.size = audio_size_;
  HrscProcess(sdk_handle_, &hrsc_buffer);
  if (save_file_ && audio_inconvert_file_.is_open()) {
    audio_inconvert_file_.write(adapter_buffer_, audio_size_);
  }
  return 0;
}

void AudioEngine::DeInitSDK() {
  RCLCPP_WARN(rclcpp::get_logger("audio_capture"), "before stop audio sdk!");
  HrscStop(sdk_handle_);
  RCLCPP_WARN(rclcpp::get_logger("audio_capture"), "stop audio sdk success!");
  HrscRelease(&sdk_handle_);
  RCLCPP_WARN(rclcpp::get_logger("audio_capture"),
              "destory audio sdk success!");
}

void AudioEngine::VoipDataCallback_(const void *cookie, const HrscCallbackData *data) {
  if (!data) return;
  RCLCPP_DEBUG(rclcpp::get_logger("audio_capture"),
               "recv hrsc sdk callback audio, angle:%f, score:%f, data size:%d",
               data->angle, data->score, data->audio_buffer.size);
  if (audio_cb_) {
    audio_cb_(reinterpret_cast<char *>(data->audio_buffer.audio_data),  // nolint
        data->audio_buffer.size);
  }
}

void AudioEngine::WakeupDataCallback_(const void *cookie, const HrscCallbackData *data,
                        const int keyword_index) {
  if (!data) return;
  std::cout << "recv hrsc sdk wakeup data , size is " << data->audio_buffer.size
            << ", key index:" << keyword_index << std::endl;
}

void AudioEngine::AsrDataCallback_(const void *cookie, const HrscCallbackData *data) {
  if (!data) return;
  static int last_state = 0;
  if (audio_asr_data_cb_) {
    if ((vad_state_ == kHrscVadStateBegin) || (vad_state_ == kHrscVadStateMiddle)) {
      if (last_state != vad_state_) {
        last_state = vad_state_;
        RCLCPP_INFO(rclcpp::get_logger("audio_capture"), "recv hrsc sdk asr data , size is %d; timestamp start:%ld,end:%ld",
                      data->audio_buffer.size,data->audio_buffer.start,data->audio_buffer.end);
      }
      if ((asr_mode_ == 0) || (asr_mode_ == 1)) {
        if (wakeup_event_ptr_ == nullptr) {
          if (cur_vad_buf_ptr_ == nullptr) {
            cur_vad_buf_ptr_ = std::make_shared<VadDataBuf_st>();
            cur_vad_buf_ptr_->start_timestamp = data->audio_buffer.start;
          }
          if ((cur_vad_buf_ptr_->pos + data->audio_buffer.size) < AUDIO_VADDATA_BUF_SIZE) {
            memcpy(&cur_vad_buf_ptr_->data_buf[cur_vad_buf_ptr_->pos], data->audio_buffer.audio_data, data->audio_buffer.size);
            cur_vad_buf_ptr_->pos += data->audio_buffer.size;
            cur_vad_buf_ptr_->end_timestamp = data->audio_buffer.start;
          }
        } else {
          while(vad_buf_queue.size() > 0) {
            auto vad_buf = vad_buf_queue.front();
            vad_buf_queue.pop();
            if (vad_buf->end_timestamp > wakeup_event_ptr_->vad_start_timestamp) {
              audio_asr_data_cb_(vad_buf->data_buf, vad_buf->pos); 
            }
          }
          if (cur_vad_buf_ptr_) {
            audio_asr_data_cb_(cur_vad_buf_ptr_->data_buf, cur_vad_buf_ptr_->pos);
            cur_vad_buf_ptr_ = nullptr;
          }
          audio_asr_data_cb_(reinterpret_cast<char *>(data->audio_buffer.audio_data), data->audio_buffer.size);
        }
      } else {
        audio_asr_data_cb_(reinterpret_cast<char *>(data->audio_buffer.audio_data), data->audio_buffer.size); 
      }
    } else if (vad_state_ == kHrscVadStateEnd) {
      last_state = vad_state_;
      if ((asr_mode_ == 0) || (asr_mode_ == 1)) {
        if (wakeup_event_ptr_) {
          auto now = std::chrono::system_clock::now();
          auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
          if ((timestamp - wakeup_event_ptr_->vad_end_timestamp) < 5 * 1000) {
            while(vad_buf_queue.size() > 0) {
              auto vad_buf = vad_buf_queue.front();
              vad_buf_queue.pop();
              if (vad_buf->end_timestamp > wakeup_event_ptr_->vad_start_timestamp) {
                audio_asr_data_cb_(vad_buf->data_buf, vad_buf->pos); 
              }
            }
            if (cur_vad_buf_ptr_) {
              audio_asr_data_cb_(cur_vad_buf_ptr_->data_buf, cur_vad_buf_ptr_->pos);
              cur_vad_buf_ptr_ = nullptr;
            }
          } else {
            wakeup_event_ptr_ = nullptr;
            cur_vad_buf_ptr_ = nullptr;
          }
        }
        if (cur_vad_buf_ptr_) {
          vad_buf_queue.push(cur_vad_buf_ptr_);
          cur_vad_buf_ptr_ = nullptr;
          if (vad_buf_queue.size() >3) {
            vad_buf_queue.pop();
          } 
        }
      }
    }
  }
}

void AudioEngine::EventCallback_(const void *cookie, HrscEventCallbackData event) {
  static int wkp_count = 0;
  RCLCPP_INFO(rclcpp::get_logger("audio_engine"), "event:%d, timestamp start:%ld--end:%ld", event.event_type, event.vad_start_timestamp, event.vad_end_timestamp);
  if (event.event_type == kHrscEventWkpNormal || event.event_type == kHrscEventWkpOneshot) {
    std::cout << "recv hrsc sdk event wakeup success, wkp count is "
              << ++wkp_count << std::endl;
    wakeup_event_ptr_ = std::make_shared<HrscEventCallbackData>();
    wakeup_event_ptr_->event_type = event.event_type;
    wakeup_event_ptr_->vad_start_timestamp = event.vad_start_timestamp;
    wakeup_event_ptr_->vad_end_timestamp = event.vad_end_timestamp;
    if (audio_event_cb_) {
      audio_event_cb_(event.event_type);
    }
  } else if (event.event_type == kHrscEventVadBegin) {
    vad_state_ = kHrscVadStateBegin;
    RCLCPP_INFO(rclcpp::get_logger("audio_engine"), "recv hrsc vad begin event");
    if (audio_event_cb_) {
      audio_event_cb_(event.event_type);
    } 
  } else if (event.event_type == kHrscEventVadEnd) {
    vad_state_ = kHrscVadStateEnd;
    RCLCPP_INFO(rclcpp::get_logger("audio_engine"), "recv hrsc vad end event");
    if (audio_event_cb_) {
      audio_event_cb_(event.event_type);
    }
  }
}

void AudioEngine::CmdDataCallback_(const void *cookie, const char *cmd) {
  if (!cmd) return;
  std::cout << "recv hrsc sdk command data: " << cmd << std::endl;
  if (audio_cmd_cb_) {
    audio_cmd_cb_(cmd);
  }
}

void AudioEngine::DoaCallback_(const void *cookie, int doa) {
  std::cout << "recv hrsc sdk doa data: " << doa << std::endl;
  if (audio_smart_cb_) {
    audio_smart_cb_(doa);
  }
}

void AudioEngine::AudioEngine::AsrCallback_(const void *cookie, const char *asr) {
  std::cout << "asr is: " << asr << std::endl;
  if (audio_asr_cb_) {
    audio_asr_cb_(asr);
  }
}
}  // namespace audio
}  // namespace hobot

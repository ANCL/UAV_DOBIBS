/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <fstream>
#include <map>
#include <vector>
#include <set>
#include <string>

#include "definitions.hpp"

#include <uORB/uORBTopics.h>

namespace px4
{

/**
 * @class Replay
 * Parses an ULog file and replays it in 'real-time'. The timestamp of each replayed message is offset
 * to match the starting time of replay. It keeps a stream for each subscription to find the next message
 * to replay. This is necessary because data messages from different subscriptions don't need to be in
 * monotonic increasing order.
 */
class Replay
{
public:
	Replay();

	/// Destructor, also waits for task exit
	~Replay();

	/// Start task.
	/// @param quiet silently fail if no log file found
	/// @param apply_params_only if true, only apply parameters from definitions section of the file
	///                          and user-overridden parameters, then exit w/o replaying.
	/// @return		OK on success.
	static int		start(bool quiet, bool apply_params_only);

	static void	task_main_trampoline(int argc, char *argv[]);

	void		task_main();

	/**
	 * Tell the replay module that we want to use replay mode.
	 * After that, only 'replay start' must be executed (typically the last step after startup).
	 * @param file_name file name of the used log replay file. Will be copied.
	 */
	static void setupReplayFile(const char *file_name);

	static bool isSetup() { return _replay_file; }
private:
	bool _task_should_exit = false;
	std::set<std::string> _overridden_params;
	std::map<std::string, std::string> _file_formats; ///< all formats we read from the file

	uint64_t _file_start_time;
	uint64_t _replay_start_time;
	std::streampos _data_section_start; ///< first ADD_LOGGED_MSG message
	std::vector<uint8_t> _read_buffer;

	struct Subscription {

		const orb_metadata *orb_meta = nullptr; ///< if nullptr, this subscription is invalid
		orb_advert_t orb_advert = nullptr;
		uint8_t multi_id;
		int timestamp_offset; ///< marks the field of the timestamp

		std::streampos next_read_pos;
		uint64_t next_timestamp; ///< timestamp of the file
	};
	std::vector<Subscription> _subscriptions;

	/** keep track of file position to avoid adding a subscription multiple times. */
	std::streampos _subscription_file_pos = 0;

	bool readFileHeader(std::ifstream &file);

	/**
	 * Read definitions section: check formats, apply parameters and store
	 * the start of the data section.
	 * @return true on success
	 */
	bool readFileDefinitions(std::ifstream &file);

	///file parsing methods. They return false, when further parsing should be aborted.
	bool readFormat(std::ifstream &file, uint16_t msg_size);
	bool readAndAddSubscription(std::ifstream &file, uint16_t msg_size);

	/**
	 * Read the file header and definitions sections. Apply the parameters from this section
	 * and apply user-defined overridden parameters.
	 * @return true on success
	 */
	bool readDefinitionsAndApplyParams(std::ifstream &file);

	/**
	 * Read and handle additional messages starting at current file position, while position < end_position.
	 * This handles dropout and parameter update messages.
	 * We need to handle these separately, because they have no timestamp. We look at the file position instead.
	 * @return false on file error
	 */
	bool readAndHandleAdditionalMessages(std::ifstream &file, std::streampos end_position);
	bool readDropout(std::ifstream &file, uint16_t msg_size);
	bool readAndApplyParameter(std::ifstream &file, uint16_t msg_size);

	/**
	 * Find next data message for this subscription, starting with the stored file offset.
	 * Skip the first message, and if found, read the timestamp and store the new file offset.
	 * This also takes care of new subscriptions and parameter updates. When reaching EOF,
	 * the subscription is set to invalid.
	 * File seek position is arbitrary after this call.
	 * @return false on file error
	 */
	bool nextDataMessage(std::ifstream &file, Subscription &subscription, int msg_id);

	static const orb_metadata *findTopic(const std::string &name);
	/** get the array size from a type. eg. float[3] -> return float */
	static std::string extractArraySize(const std::string &type_name_full, int &array_size);
	/** get the size of a type that is not an array */
	static size_t sizeOfType(const std::string &type_name);
	/** get the size of a type that can be an array */
	static size_t sizeOfFullType(const std::string &type_name_full);

	void setUserParams(const char *filename);

	static char *_replay_file;
};

} //namespace px4

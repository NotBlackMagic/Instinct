#pragma once

#include <stdarg.h>
#include <stdio.h>

#include "logger.hpp"

class Shell {
	public:
		typedef bool (*CommandHandler)(const char* args);

		struct CommandEntry {
			const char* name;
			CommandHandler handler;
			const char* helpText;
		};

		// Wrapper for a group of commands (i.e. each module registers a group of commands)
		struct CommandList {
			const CommandEntry* commandGroup;
			CommandList* next;
		};

		Shell();

		/// @brief Initializes the shell (Empty function).
		static void Init();

		/// @brief Input received data to the shell to be interpreted/executed.
		/// @param data Pointer to received data.
		/// @param len	Length of received data.
		void Input(uint8_t* data, uint16_t len);

		/// @brief Register/add a new shell command(s) and handler.
		/// @param list Pointer to static list of shell command group node to be registered/added.
		/// @param len	Pointer to static array of all command entries to be registered/added for the new group node.
		static void RegisterCommands(CommandList* list, const CommandEntry* array);

		/// @brief Returns head of command list registry i.e. returns link to full command list.
		/// @return Pointer to list head.
		static const CommandList* GetRegistry() { return head; }

	private:
		// Command interpretation tracking variables (each line)
		static constexpr int bufferSize = 64;
		char lineBuffer[bufferSize];
		int lineIndex;

		// List head of command registry
		static CommandList* head;

		// Internal helper to parse and execute commands
		void Execute();
		void ParseArgs(char* cmdLine);
};
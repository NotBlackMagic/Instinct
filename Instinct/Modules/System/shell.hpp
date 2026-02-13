#pragma once

#include <stdarg.h>
#include <stdio.h>

#include "logger.hpp"

void RegisterSystemCommands();

class Shell {
	public:
		Shell();

		static void Init();
		void Input(uint8_t* data, uint16_t len);

		typedef bool (*CommandHandler)(const char* args);

		struct CommandEntry {
			const char* name;
			CommandHandler handler;
			const char* helpText;
		};

		//Wrapper for a group of commands (i.e. each module registers a group of commands)
		struct CommandList {
			const CommandEntry* commandGroup;
			CommandList* next;
		};

		static void RegisterCommands(CommandList* list, const CommandEntry* array);

		static const CommandList* GetRegistry() { return head; }
	private:
		void Execute();
		void ParseArgs(char* cmdLine);

		static constexpr int BufferSize = 64;
		char lineBuffer[BufferSize];
		int lineIndex;

		static CommandList* head;
};
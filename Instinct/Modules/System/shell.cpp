/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/System/shell.cpp
 */

#include "shell.hpp"

Shell::CommandList* Shell::head = nullptr;

Shell::Shell() : lineIndex(0) {
	memset(lineBuffer, 0, bufferSize);
}

void Shell::Init() {
	
}

void Shell::Input(uint8_t* data, uint16_t len) {
	uint16_t i;
	for(i = 0; i < len; i++) {
		uint8_t c = data[i];

		// Check for BACKSPACE
		if(c == 127 || c == 8) {
			if(lineIndex > 0) {
				lineIndex -= 1;

				// Echo backspace
				Logger::Instance().Write("\b \b");
			}
			continue;
		}

		// Check for ENTER
		if(c == '\r' || c == '\n') {
			if(lineIndex > 0) {
				lineBuffer[lineIndex] = 0;

				// Newline
				Logger::Instance().Write("\r\n");

				Execute();
				lineIndex = 0;
			}
			else {
				Logger::Instance().Write("\r\n");
			}
			continue;
		}

		// Else fill to buffer
		if(lineIndex < (bufferSize - 1) && c >= 32 && c <= 126) {
			lineBuffer[lineIndex++] = c;

			// Echo read byte/char
			Logger::Instance().Write((char*)&c, 1);
		}
	}
}

void Shell::Execute() {
	// Split command and arguments

	char* cmd = lineBuffer;
	char* args = nullptr;

	// Find first space (after command name)
	char* space = strchr(lineBuffer, ' ');
	if(space) {
		*space = '\0'; 		// Null terminate the command
		args = space + 1; 	// Args start after space
		
		//Skip extra spaces in args
		while (*args == ' ') {
			args++;
		}
	}

	if(strlen(cmd) == 0) {
		return;
	}

	// Iterate through command list (modules) where each item is itself a list of commands
	const Shell::CommandList* currentList = head;

	while(currentList != nullptr) {
		// Iterate through commands in each list (module)
		const CommandEntry* entry = currentList->commandGroup;

		while(entry->name != nullptr) {
			if (strcmp(cmd, entry->name) == 0) {
				// Found! Run it.
				if (!entry->handler(args)) {
					Logger::Instance().Write("Command Error\r\n");
				}
				return;
			}
			entry++;	// Go to next command in this list/array
		}

		currentList = currentList->next;
	}

	Logger::Instance().Printf("Unknown command: '%s'. Type 'help'.\r\n", cmd);
}

void Shell::RegisterCommands(CommandList *list, const CommandEntry *array) {
	//Setup new command group/array node
	list->commandGroup = array;

	//Update overall list i.e. add new command to front of list
	list->next = head;
	head = list;
}
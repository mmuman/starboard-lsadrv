/*==========================================================================
 * Linux kernel driver for eIT-Xiroku optical touch sensor header file
 *
 * File: lsadrv-vkey.c
 *
 * Purpose:
 *    Definitions of Microsoft Windows virtual key code
 *
 * Copyright (C) 2009  eIT Co., Ltd. and Xiroku Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
============================================================================*/

#include <linux/input.h>
#include "lsadrv-ioctl.h"
#include "lsadrv-vkey.h"

static int keylist[] = {
	KEY_ESC, KEY_MINUS, KEY_EQUAL, KEY_BACKSPACE, KEY_TAB,
	KEY_1, KEY_2, KEY_3, KEY_4, KEY_5, KEY_6, KEY_7, KEY_8, KEY_9, KEY_0,
	KEY_A, KEY_B, KEY_C, KEY_D, KEY_E, KEY_F, KEY_G, KEY_H, KEY_I, KEY_J,
	KEY_K, KEY_L, KEY_M, KEY_N, KEY_O, KEY_P, KEY_Q, KEY_R, KEY_S, KEY_T,
	KEY_U, KEY_V, KEY_W, KEY_X, KEY_Y, KEY_Z,
	KEY_F1, KEY_F2, KEY_F3, KEY_F4, KEY_F5, KEY_F6, KEY_F7, KEY_F8, KEY_F9, KEY_F10,
	KEY_F11, KEY_F12, KEY_F13, KEY_F14, KEY_F15, KEY_F16, KEY_F17, KEY_F18, KEY_F19, KEY_F20,
	KEY_F21, KEY_F22, KEY_F23, KEY_F24,
	KEY_LEFTBRACE, KEY_RIGHTBRACE, KEY_ENTER, KEY_LEFTCTRL, KEY_SEMICOLON,
	KEY_APOSTROPHE, KEY_GRAVE, KEY_LEFTSHIFT, KEY_BACKSLASH, KEY_COMMA,
	KEY_DOT, KEY_SLASH, KEY_RIGHTSHIFT, KEY_KPASTERISK, KEY_LEFTALT,
	KEY_SPACE, KEY_CAPSLOCK, KEY_NUMLOCK, KEY_SCROLLLOCK, KEY_KPMINUS,
	KEY_KP0, KEY_KP1, KEY_KP2, KEY_KP3, KEY_KP4, KEY_KP5, KEY_KP6, KEY_KP7, KEY_KP8, KEY_KP9,
	KEY_KPPLUS, KEY_KPDOT, KEY_102ND, KEY_KPENTER, KEY_RIGHTCTRL, KEY_KPSLASH, KEY_SYSRQ,
	KEY_RIGHTALT, KEY_LINEFEED, KEY_HOME, KEY_UP, KEY_PAGEUP, KEY_LEFT, KEY_RIGHT, KEY_END,
	KEY_DOWN, KEY_PAGEDOWN, KEY_INSERT, KEY_DELETE, KEY_MACRO, KEY_MUTE, KEY_VOLUMEDOWN,
	KEY_VOLUMEUP, KEY_POWER, KEY_KPEQUAL, KEY_KPPLUSMINUS, KEY_PAUSE,
	KEY_KPCOMMA, KEY_LEFTMETA, KEY_RIGHTMETA, KEY_COMPOSE, KEY_HELP
};

/*
 * get key list
 *   return: number of keys
 */
int lsadrv_get_key_list(const int **list)
{
	*list = keylist;
	return (sizeof(keylist) / sizeof(keylist[0]));
}

/* convert vertual key code to key code defined in input subsystem */
int lsadrv_vkeytokey(int vkey, int extended)
{
	int key = 0;

	switch (vkey)
	{
		case VK_LBUTTON:		key = BTN_LEFT;		break;
		case VK_RBUTTON:		key = BTN_RIGHT;	break;
		case VK_CANCEL:			break;
		case VK_MBUTTON:		key = BTN_MIDDLE;	break;

		case VK_XBUTTON1:		break;
		case VK_XBUTTON2:		break;

		case VK_BACK:			key = KEY_BACKSPACE;	break;
		case VK_TAB:			key = KEY_TAB;		break;
		case VK_RETURN:			key = KEY_ENTER;	break;

		case VK_CLEAR:			break;

		case VK_SHIFT:			key = extended ? KEY_RIGHTSHIFT : KEY_LEFTSHIFT;	break;
		case VK_CONTROL:		key = extended ? KEY_RIGHTCTRL : KEY_LEFTCTRL;		break;
		case VK_MENU:			key = extended ? KEY_RIGHTALT : KEY_LEFTALT;		break;
		case VK_PAUSE:			key = KEY_PAUSE;	break;
		case VK_CAPITAL:		key = KEY_CAPSLOCK;	break;

		case VK_KANA:			break;
		case VK_JUNJA:			break;
		case VK_FINAL:			break;
		case VK_KANJI:			break;

		case VK_ESCAPE: 		key = KEY_ESC;		break;

		case VK_CONVERT:		break;
		case VK_NONCONVERT:		break;
		case VK_ACCEPT:			break;
		case VK_MODECHANGE:		break;

		case VK_SPACE:			key = KEY_SPACE;	break;
		case VK_PRIOR:			key = KEY_PAGEUP;	break;
		case VK_NEXT:			key = KEY_PAGEDOWN;	break;
		case VK_END:			key = KEY_END;		break;
		case VK_HOME:			key = KEY_HOME;		break;
		case VK_LEFT:			key = KEY_LEFT;		break;
		case VK_UP:			key = KEY_UP;		break;
		case VK_RIGHT:			key = KEY_RIGHT;	break;
		case VK_DOWN:			key = KEY_DOWN;		break;
		case VK_SELECT:			break;
		case VK_PRINT:			break;
		case VK_EXECUTE:		break;
		case VK_SNAPSHOT:		break;
		case VK_INSERT:			key = KEY_INSERT;	break;
		case VK_DELETE:			key = KEY_DELETE;	break;
		case VK_HELP:			key = KEY_HELP;		break;

		/* VK_0 - VK_9 are the same as ASCII '0' - '9' (0x30 - 0x39) */
		case VK_0:			key = KEY_0;		break;
		case VK_1:			key = KEY_1;		break;
		case VK_2:			key = KEY_2;		break;
		case VK_3:			key = KEY_3;		break;
		case VK_4:			key = KEY_4;		break;
		case VK_5:			key = KEY_5;		break;
		case VK_6:			key = KEY_6;		break;
		case VK_7:			key = KEY_7;		break;
		case VK_8:			key = KEY_8;		break;
		case VK_9:			key = KEY_9;		break;

		/* VK_A - VK_Z are the same as ASCII 'A' - 'Z' (0x41 - 0x5A) */
		case VK_A:			key = KEY_A;		break;
		case VK_B:			key = KEY_B;		break;
		case VK_C:			key = KEY_C;		break;
		case VK_D:			key = KEY_D;		break;
		case VK_E:			key = KEY_E;		break;
		case VK_F:			key = KEY_F;		break;
		case VK_G:			key = KEY_G;		break;
		case VK_H:			key = KEY_H;		break;
		case VK_I:			key = KEY_I;		break;
		case VK_J:			key = KEY_J;		break;
		case VK_K:			key = KEY_K;		break;
		case VK_L:			key = KEY_L;		break;
		case VK_M:			key = KEY_M;		break;
		case VK_N:			key = KEY_N;		break;
		case VK_O:			key = KEY_O;		break;
		case VK_P:			key = KEY_P;		break;
		case VK_Q:			key = KEY_Q;		break;
		case VK_R:			key = KEY_R;		break;
		case VK_S:			key = KEY_S;		break;
		case VK_T:			key = KEY_T;		break;
		case VK_U:			key = KEY_U;		break;
		case VK_V:			key = KEY_V;		break;
		case VK_W:			key = KEY_W;		break;
		case VK_X:			key = KEY_X;		break;
		case VK_Y:			key = KEY_Y;		break;
		case VK_Z:			key = KEY_Z;		break;

		case VK_LWIN:			break;
		case VK_RWIN:			break;
		case VK_APPS:			key = KEY_MENU;		break;	/*right menu key*/

		case VK_SLEEP:			key = KEY_SLEEP;	break;

		case VK_NUMPAD0:		key = KEY_KP0;		break;
		case VK_NUMPAD1:		key = KEY_KP1;		break;
		case VK_NUMPAD2:		key = KEY_KP2;		break;
		case VK_NUMPAD3:		key = KEY_KP3;		break;
		case VK_NUMPAD4:		key = KEY_KP4;		break;
		case VK_NUMPAD5:		key = KEY_KP5;		break;
		case VK_NUMPAD6:		key = KEY_KP6;		break;
		case VK_NUMPAD7:		key = KEY_KP7;		break;
		case VK_NUMPAD8:		key = KEY_KP8;		break;
		case VK_NUMPAD9:		key = KEY_KP9;		break;
		case VK_MULTIPLY:		key = KEY_KPASTERISK;	break;
		case VK_ADD:			key = KEY_KPPLUS;	break;
		case VK_SEPARATOR:		key = extended ? KEY_KPCOMMA : KEY_COMMA;	break;
		case VK_SUBTRACT:		key = extended ? KEY_KPMINUS : KEY_MINUS;	break;
		case VK_DECIMAL:		key = extended ? KEY_KPDOT : KEY_DOT;		break;
		case VK_DIVIDE:			key = extended ? KEY_SLASH : KEY_KPSLASH;	break;

		case VK_F1:			key = KEY_F1;		break;
		case VK_F2:			key = KEY_F2;		break;
		case VK_F3:			key = KEY_F3;		break;
		case VK_F4:			key = KEY_F4;		break;
		case VK_F5:			key = KEY_F5;		break;
		case VK_F6:			key = KEY_F6;		break;
		case VK_F7:			key = KEY_F7;		break;
		case VK_F8:			key = KEY_F8;		break;
		case VK_F9:			key = KEY_F9;		break;
		case VK_F10:			key = KEY_F10;		break;
		case VK_F11:			key = KEY_F11;		break;
		case VK_F12:			key = KEY_F12;		break;
		case VK_F13:			key = KEY_F13;		break;
		case VK_F14:			key = KEY_F14;		break;
		case VK_F15:			key = KEY_F15;		break;
		case VK_F16:			key = KEY_F16;		break;
		case VK_F17:			key = KEY_F17;		break;
		case VK_F18:			key = KEY_F18;		break;
		case VK_F19:			key = KEY_F19;		break;
		case VK_F20:			key = KEY_F20;		break;
		case VK_F21:			key = KEY_F21;		break;
		case VK_F22:			key = KEY_F22;		break;
		case VK_F23:			key = KEY_F23;		break;
		case VK_F24:			key = KEY_F24;		break;

		case VK_NUMLOCK:		key = KEY_NUMLOCK;	break;
		case VK_SCROLL:			key = KEY_SCROLLLOCK;	break;

		case VK_OEM_NEC_EQUAL:		key = KEY_KPEQUAL;	break;	/* '=' key on numpad */

		case VK_LSHIFT:			key = KEY_LEFTSHIFT;	break;
		case VK_RSHIFT:			key = KEY_RIGHTSHIFT;	break;
		case VK_LCONTROL:		key = KEY_LEFTCTRL;	break;
		case VK_RCONTROL:		key = KEY_RIGHTCTRL;	break;
		case VK_LMENU:			key = KEY_LEFTALT;	break;
		case VK_RMENU:			key = KEY_RIGHTALT;	break;

		case VK_BROWSER_BACK:		break;
		case VK_BROWSER_FORWARD:	break;
		case VK_BROWSER_REFRESH:	break;
		case VK_BROWSER_STOP:		break;
		case VK_BROWSER_SEARCH:		break;
		case VK_BROWSER_FAVORITES:	break;
		case VK_BROWSER_HOME:		break;

		case VK_VOLUME_MUTE:		break;
		case VK_VOLUME_DOWN:		break;
		case VK_VOLUME_UP:		break;
		case VK_MEDIA_NEXT_TRACK:	break;
		case VK_MEDIA_PREV_TRACK:	break;
		case VK_MEDIA_STOP:		break;
		case VK_MEDIA_PLAY_PAUSE:	break;
		case VK_LAUNCH_MAIL:		break;
		case VK_LAUNCH_MEDIA_SELECT:	break;
		case VK_LAUNCH_APP1:		break;
		case VK_LAUNCH_APP2:		break;

		case VK_OEM_1:			break;	/* ';:' for US */
		case VK_OEM_PLUS:		break;	/* '+' any country */
		case VK_OEM_COMMA:		break;	/* ',' any country */
		case VK_OEM_MINUS:		break;	/* '-' any country */
		case VK_OEM_PERIOD:		break;	/* '.' any country */
		case VK_OEM_2:			break;		/* '/?' for US */
		case VK_OEM_3:			break;	/* '`~' for US */

		case VK_OEM_4:			key = KEY_LEFTBRACE;	break;	/*  '[{' for US */
		case VK_OEM_5:			key = KEY_BACKSLASH;	break;	/*  '\|' for US */
		case VK_OEM_6:			key = KEY_RIGHTBRACE;	break;	/*  ']}' for US */
		case VK_OEM_7:			key = KEY_APOSTROPHE;	break;	/*  ''"' for US */

		case VK_OEM_8:			break;

		case VK_OEM_AX:			break;	/*  'AX' key on Japanese AX kbd */
		case VK_OEM_102:		break;	/*  "<>" or "\|" on RT 102-key kbd. */
		case VK_ICO_HELP:		break;	/*  Help key on ICO */
		case VK_ICO_00:			break;	/*  00 key on ICO */

		case VK_PROCESSKEY:		break;

		case VK_ICO_CLEAR:		break;

		case VK_PACKET:			break;

		case VK_OEM_RESET:		break;
		case VK_OEM_JUMP:		break;
		case VK_OEM_PA1:		break;
		case VK_OEM_PA2:		break;
		case VK_OEM_PA3:		break;
		case VK_OEM_WSCTRL:		break;
		case VK_OEM_CUSEL:		break;
		case VK_OEM_ATTN:		break;
		case VK_OEM_FINISH:		break;
		case VK_OEM_COPY:		break;
		case VK_OEM_AUTO:		break;
		case VK_OEM_ENLW:		break;
		case VK_OEM_BACKTAB:		break;

		case VK_ATTN:			break;
		case VK_CRSEL:			break;
		case VK_EXSEL:			break;
		case VK_EREOF:			break;
		case VK_PLAY:			break;
		case VK_ZOOM:			break;
		case VK_NONAME:			break;
		case VK_PA1:			break;
		case VK_OEM_CLEAR:		break;

		default:			break;
	}
	return key;
}


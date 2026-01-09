//------------------------------------------------------------------------------
// This is Open source software. You can place this code on your site, but don't
// forget a link to my YouTube-channel: https://www.youtube.com/channel/UChButpZaL5kUUl_zTyIDFkQ
// ��� ����������� ����������� ���������������� ��������. �� ������ ���������
// ��� �� ����� �����, �� �� �������� ������� ������ �� ��� YouTube-����� 
// "����������� � ���������" https://www.youtube.com/channel/UChButpZaL5kUUl_zTyIDFkQ
// �����: �������� ������ / Nadyrshin Ruslan
//------------------------------------------------------------------------------
#ifndef _F16F_H
#define _F16F_H

//#include <types.h>
#include <stdint.h> 

// ����� ������������, 6�8 ��������
#define f10x16_FLOAT_WIDTH         10
#define f10x16_FLOAT_HEIGHT        16

// ���-�� �������� � ������� ������
#define f10x16f_NOFCHARS           256

// ������� ���������� ��������� �� ���������� ������� Char
uint8_t* f10x16f_GetCharTable(uint8_t Char);

#endif 

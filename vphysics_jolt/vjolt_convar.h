
#pragma once

#include "tier1/tier1.h"

#ifndef JOLTCONVAR_H
#define JOLTCONVAR_H

#ifdef GAME_GMOD
class JoltConVar : public ConCommandBase
{
friend class CCvar;

public:
	typedef ConCommandBase BaseClass;

	JoltConVar( const char *pName, const char *pDefaultValue, int flags = 0, const char *pHelpString = "", bool bMin = false, float fMin = 0.0, bool bMax = false, float fMax = 0.0 );

	virtual ~JoltConVar( void );

	virtual	bool IsCommand( void ) const { return true; };

	virtual int AutoCompleteSuggest( const char *partial, CUtlVector< CUtlString > &commands );

	virtual bool CanAutoComplete( void ) { return true; };

	virtual void Dispatch( const CCommand &command );
public:
	virtual float GetFloat(){ return m_fValue; }
	virtual int GetInt(){ return m_nValue; }
	virtual bool GetBool() { return !!GetInt(); }
	virtual const char* GetString() { return m_pszString; }

	virtual void SetValue(const char*);
private:
	union
	{
		FnCommandCallbackVoid_t m_fnCommandCallbackV1;
		FnCommandCallback_t m_fnCommandCallback;
		ICommandCallback *m_pCommandCallback; 
	};

	union
	{
		FnCommandCompletionCallback	m_fnCompletionCallback;
		ICommandCompletionCallback *m_pCommandCompletionCallback;
	};

	bool m_bHasCompletionCallback : 1;
	bool m_bUsingNewCommandCallback : 1;
	bool m_bUsingCommandCallbackInterface : 1;
private:
	virtual void SetValueInternal(const char*);

	const char* m_pszName;
	const char *m_pszDefaultValue;
	
	char *m_pszString;
	int m_StringLength;

	float m_fValue;
	int m_nValue;

	bool m_bHasMin;
	float m_fMinVal;
	bool m_bHasMax;
	float m_fMaxVal;
};
#else
	#define JoltConVar ConVar
#endif
#endif
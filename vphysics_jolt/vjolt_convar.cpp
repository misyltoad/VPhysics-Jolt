#include "cbase.h"

#ifdef GAME_GMOD
JoltConVar::JoltConVar( const char *pName, const char *pDefaultValue, int flags, const char *pHelpString, bool bMin, float fMin, bool bMax, float fMax )
{
	// ConVar stuff
	m_pszName = pName;
	m_pszDefaultValue = pDefaultValue;

	SetValue(m_pszDefaultValue);

	m_bHasMin = bMin;
	m_fMinVal = fMin;
	m_bHasMax = bMax;
	m_fMaxVal = fMax;

	// Set the callback
	m_bUsingNewCommandCallback = true;
	//m_fnCompletionCallback = JoltConVar_CompletionFunc;
	m_bHasCompletionCallback = ( m_fnCompletionCallback != 0 );
	m_bUsingCommandCallbackInterface = false;

	// Setup the rest
	BaseClass::CreateBase( pName, pHelpString, flags );
}

JoltConVar::~JoltConVar( void )
{
	if (m_pszString) {
		delete[] m_pszString;
	}
}

void JoltConVar::Dispatch( const CCommand &cmd )
{
	if ( cmd.ArgC() == 1 ) {
		Msg( "%s: %s\n", m_pszName, GetString() );
	} else if ( cmd.ArgC() == 2 ) {
		SetValue( cmd.Arg(1) );
		Msg( "%s set to %s\n", m_pszName, GetString() );
	}
}

int	JoltConVar::AutoCompleteSuggest( const char *partial, CUtlVector< CUtlString > &commands )
{
	std::string cmd = partial;
	size_t found = cmd.find_first_of( " " );
	if ( found ) {
		cmd = cmd.substr(0, found);
	}

	commands.AddToTail(( cmd + " " + GetString() ).c_str() );
	return 1;
}

void JoltConVar::SetValueInternal( const char* m_pszValue )
{
	if (m_pszString) {
		delete[] m_pszString;
	}

	m_StringLength = V_strlen( m_pszValue ) + 1;
	m_pszString = new char[m_StringLength];
	memcpy( m_pszString, m_pszValue, m_StringLength );

	m_fValue = ( float )atof( m_pszString );
	m_nValue = atoi( m_pszString );
}

void JoltConVar::SetValue( const char* m_pszValue )
{
	SetValueInternal(m_pszValue);

	float m_fClamped = m_fValue;
	if (m_bHasMin) {
		m_fClamped = clamp(m_fValue, m_fMinVal, m_fValue);
	}

	if (m_bHasMax) {
		m_fClamped = clamp(m_fValue, m_fValue, m_fMaxVal);
	}

	if (m_fClamped != m_fValue) {
		SetValueInternal(std::to_string(m_fClamped).c_str());
	}
}
#endif
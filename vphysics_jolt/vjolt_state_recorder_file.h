
#pragma once

class JoltStateRecorderFile final : public JPH::StateRecorder
{
public:
	JoltStateRecorderFile( const char* pszPath, bool input )
		: m_Stream( pszPath, std::ios::binary | ( input ? std::ios::in : std::ios::out ) )
	{
	}

	JoltStateRecorderFile( JoltStateRecorderFile &&other )
		: StateRecorder( other )
		, m_Stream( std::move( other.m_Stream ) )
	{
	}

	void WriteBytes( const void* inData, size_t inNumBytes ) override
	{
		m_Stream.write( reinterpret_cast< const char * >( inData ), inNumBytes );
	}

	void ReadBytes( void* outData, size_t inNumBytes ) override
	{
		m_Stream.read( reinterpret_cast< char * >( outData ), inNumBytes );
	}

	bool IsEOF() const override		{ return m_Stream.eof(); }
	bool IsFailed() const override	{ return m_Stream.fail(); }

	bool IsValid() const			{ return !m_Stream.bad(); }

private:
	std::fstream m_Stream;
};

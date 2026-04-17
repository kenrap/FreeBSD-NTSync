PORTNAME=	ntsync-kmod
DISTVERSION=	g20260416
CATEGORIES=	graphics

MAINTAINER=	tmp386@live.com
COMMENT=	NTSync kernel driver for Windows compatibility layers
WWW=		https://github.com/XaeroVincent/FreeBSD-NTSync

LICENSE=	GPLv2
LICENSE_FILE=	${WRKSRC}/LICENSE

USES=		compiler:c11

USE_GITHUB=	yes
GH_ACCOUNT=	XaeroVincent
GH_PROJECT=	FreeBSD-NTSync
GH_TAGNAME=	e9497ee2309526cd078dfae3b8b14b3cbf4bdac4

do-build:
	cd ${WRKSRC}; make COMPAT32=yes

do-install:
	${MKDIR} ${STAGEDIR}${KMODDIR}
	${MKDIR} ${STAGEDIR}${PREFIX}/include/linux
	${INSTALL_KLD} ${WRKSRC}/ntsync.ko ${STAGEDIR}${KMODDIR}
	${INSTALL_DATA} ${WRKSRC}/ntsync.h ${STAGEDIR}${PREFIX}/include/linux

.include <bsd.port.mk>
